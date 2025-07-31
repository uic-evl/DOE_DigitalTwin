from __future__ import annotations

import math
import torch
from collections.abc import Sequence
import time
import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import spawn_from_usd, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import sample_uniform
from isaaclab.assets import ArticulationCfg, Articulation
from isaaclab.assets import AssetBaseCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.actuators import IdealPDActuatorCfg, IdealPDActuator
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets import RigidObject, RigidObjectCfg, RigidObjectCollection, RigidObjectCollectionCfg 
from isaaclab.managers import SceneEntityCfg

from collections import OrderedDict
import numpy as np

@configclass
class HotCellEnvCfg(DirectRLEnvCfg):
    # simulation runs 100 physics steps for each environment step
    decimation = 100
    
    # each episode runs for 6 seconds max
    episode_length_s = 6

    # scales the action values that are sent to the robot
    action_scale = 1.0

    # number of values in the observation space
    observation_space = 10

    # simulation setup with custom friction and timestep
    sim: SimulationCfg = SimulationCfg(dt=1/240, render_interval=4,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ))

    # path to the usd file for assets in the scene
    assets_usd_path = r"C:/dev/DT_stuff_OV/unity_connection/hot_cell.usd"
    assets_usd_cfg = UsdFileCfg()
    assets_usd_cfg.usd_path = assets_usd_path

    # names of the target cubes for left and right arm
    object_info_dict = {
        "left_cube":{"asset_name":"mock_lab/GrabCubeL"}, 
        "right_cube":{"asset_name":"mock_lab/GrabCubeR"}
    }

    # config to load the cubes into the scene
    object_collection: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects={
            object_name:RigidObjectCfg(
                prim_path=f"/World/envs/env_.*/assets/{object_info['asset_name']}",
                debug_vis=True
            )
            for object_name, object_info in object_info_dict.items()
        }
    )

    # config for both robot arms, including their workspace area and assigned target cube
    robot_arm_info_dict = OrderedDict([
        ("left_arm", {
            "asset_name": "Luck_Sim",
            "workspace": {
                "x":(0.05, 0.35),
                "y":(-0.36, -0.28)
            },
            "target": "left_cube"
        }), 
        ("right_arm", {
            "asset_name": "Risk_Sim",
            "workspace":{
                "x": (0.80, 1.10),
                "y":(-0.36, -0.28)
            },
            "target": "right_cube"
        })
    ])

    # maps arm names to their joints and body configs
    arm_entities = {
        arm_name: SceneEntityCfg(
            arm_name,
            joint_names = [".*"],
            body_names=[".*"]
        )
        for arm_name, arm_info in robot_arm_info_dict.items()
    }

    # Helper function to build the configuration for robot arms
    @staticmethod
    def build_robot_arm_cfg(robot_arm_info_dict): 
        robot_arms: dict[str, ArticulationCfg] = {
            arm_name: ArticulationCfg(
                prim_path=f"/World/envs/env_.*/assets/{arm_info['asset_name']}",
                actuators={
                    # bottom joints control the shoulder and base of the arm
                    "bottom_joints":ImplicitActuatorCfg(
                        joint_names_expr = ["joint[1-3].*"], 
                        effort_limit_sim = 1000,
                        velocity_limit_sim = 2,
                        stiffness=5000,
                        damping=2*math.sqrt(5000)
                    ),
                    # middle joints handle the elbow/mid section
                    "middle_joints":ImplicitActuatorCfg(
                        joint_names_expr = ["joint[4-6].*"], 
                        effort_limit_sim = 1000,
                        velocity_limit_sim = 2,
                        stiffness=3000,
                        damping=2*math.sqrt(3000)
                    ),
                    # top joint is the wrist
                    "top_joints":ImplicitActuatorCfg(
                        joint_names_expr = ["joint7.*"], 
                        effort_limit_sim = 1000,
                        velocity_limit_sim = 2,
                        stiffness=2000,
                        damping=2*math.sqrt(2000)
                    ),
                    # gripper joints control the end effector
                    "gripper_joints":ImplicitActuatorCfg(
                        joint_names_expr = ["gripper.*"], 
                        effort_limit_sim = 1000,
                        velocity_limit_sim = 2,
                        stiffness=5000,
                        damping=2*math.sqrt(5000)
                    )
                },
                debug_vis = True
            )
            for arm_name, arm_info in robot_arm_info_dict.items()
        }

        return robot_arms 
    
    # number of parallel environments to run
    num_envs = 250

    # configure both robot arms using helper function
    robot_arms = build_robot_arm_cfg.__func__(robot_arm_info_dict)
    
    # no sensors configured yet
    sensor_types = None

    # both arms have 7 base joints each
    arm_num_base_joints = 7

    # action space: 2 arms × (7 joints + 1 gripper) = 16
    action_space = 2 * (arm_num_base_joints + 1)

    # observation space for now is just the joint info, will add sensors later
    observation_space = 2 * arm_num_base_joints

    # setup environment spacing and collision rules
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=num_envs, env_spacing=1.5, replicate_physics=False)
    scene.filter_collisions = True

    # TODO: implement rewards later

# choose device based on hardware
device = "cuda" if torch.cuda.is_available() else "cpu"

# Main environment class that extends DirectRLEnv
class HotCellEnv(DirectRLEnv):
    cfg: HotCellEnvCfg

    def __init__(self, cfg: HotCellEnvCfg, render_mode: str | None = None, **kwargs):
        self.action_scale = cfg.action_scale
        self.arm_num_base_joints = cfg.arm_num_base_joints
        self.arm_names = list(cfg.robot_arm_info_dict.keys())
        self.object_names = list(cfg.object_info_dict.keys())
        super().__init__(cfg, render_mode, **kwargs)

    def close(self):
        super().close()


    # Handles initial setup of the environment scene including spawning assets, lights, arms, and cubes
    def _setup_scene(self):
        # note: hardcoded joint ids
        self._joint_ids = (0,1,2,3,4,5,6,10)

        # stores target goal pose for each object per environment
        self._goal_pose_w = torch.zeros((self.cfg.num_envs, len(self.arm_names), 7), device=device)
        
        # add ground plane to all environments
        print("Adding Ground Plane")
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())

        # add ambient dome lighting to scene
        print("Adding lights")
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

        # replicate each environment instance across the grid
        print("Cloning Enviornments")
        self.scene.clone_environments(copy_from_source=False)

        # add the usd scene with all fixed assets
        print("Adding Hot Cells")
        spawn_from_usd(prim_path=f"/World/envs/env_.*/assets", cfg=self.cfg.assets_usd_cfg)

        # add and initialize the robotic arms
        print("Adding Arm Articulations")
        self.arms = {
            arm_name: Articulation(self.cfg.robot_arms[arm_name])
            for arm_name in self.arm_names
        }

        # create rigid objects (cubes)
        self._arm_entities_initalized = False 
        self.objects = RigidObjectCollection(self.cfg.object_collection)

    # Scales and stores the incoming action tensor before physics stepping
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        # scale actions and save them
        self.actions = self.action_scale * actions.clone()

    # Applies the scaled actions to each robot arm in the simulation
    def _apply_action(self) -> None:
        # create a padded tensor to hold all joint and gripper actions
        padded_actions = torch.zeros(self.cfg.num_envs, 2, self.arm_num_base_joints + 1)
        padded_actions = self.actions[:, :, :8]

        # apply the actions to both arms
        for arm_index in range(len(self.arm_names)):
            self.arms[self.arm_names[arm_index]].set_joint_position_target(padded_actions[:, arm_index, :], self._joint_ids)
            self.arms[self.arm_names[arm_index]].write_data_to_sim()

    # Helper: convert a world frame pose to environment frame
    def __state_w_to_state_env(self, state_w, env_ids=None) -> torch.Tensor:
        if env_ids is None:
            env_ids = torch.arange(self.scene.num_envs)
        state_w = state_w.clone()
        state_w[:, :, :3] -= self.scene.env_origins[env_ids].unsqueeze(1)
        return state_w

    # Helper: convert an environment frame pose to world frame
    def __state_env_to_state_w(self, state_w, env_ids=None) -> torch.Tensor:
        if env_ids is None:
            env_ids = torch.arange(self.scene.num_envs)
        state_w = state_w.clone()
        state_w[:, :, :3] += self.scene.env_origins[env_ids].unsqueeze(1)
        return state_w

    # Gathers all the latest observations from the arms and cubes and returns them as a dictionary
    def _get_observations(self) -> dict:
        # update internal buffer for rigid objects
        self.objects.update(dt=self.step_dt)
        
        objs_root_pose_w = torch.cat((self.objects.data.object_link_pos_w, self.objects.data.object_link_quat_w), dim=2)

        # gather all relevant object information
        object_obs = {
            "idxs": {
                object_name:self.objects.find_objects(object_name)[0][0]
                for object_name in self.object_names
            },
            "state": self.objects.data.object_state_w,
            "root_pose_w": objs_root_pose_w,
            "goal_pose_w": self._goal_pose_w,
            "root_pose_env": self.__state_w_to_state_env(objs_root_pose_w)
        }
        
        # update internal buffer for all robot arms
        for arm in self.arms.values():
            arm.update(dt=self.step_dt)

        # collect observations from each arm
        arm_obs = {
            arm_name:{
                "joint_pos": arm.data.joint_pos[:, self.cfg.arm_entities[arm_name].joint_ids], 
                "root_pose_w": torch.cat((arm.data.root_link_pos_w, arm.data.root_link_quat_w), dim=1),
                "ee_pose_w": torch.cat((arm.data.body_pos_w[:, 7], arm.data.body_quat_w[:, 7]), dim=1)
            }
            for arm_name, arm in self.arms.items()
        }

        # return full observation dictionary
        return {
            "objects": object_obs,
            "arms": arm_obs
        }

    def _get_rewards(self) -> torch.Tensor:
        # not implemented yet
        pass

    #  Returns whether the episode is terminated or truncated based on max episode length
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # currently only checking if max episode length is reached
        terminated = torch.zeros(self.cfg.num_envs)
        truncated = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, truncated

    # Helper: generate random numbers within a given range
    def _get_bounded_rand(self, lower_bound, upper_bound, shape):
        return lower_bound + (upper_bound - lower_bound) * torch.rand(shape, device=device)

    # Pick random (x, y) values inside the arm's workspace
    def _get_rand_point_in_workspace(self, workspace, env_ids):
        rand_x = self._get_bounded_rand(lower_bound=workspace["x"][0], upper_bound=workspace["x"][1], shape=env_ids.shape)
        rand_y = self._get_bounded_rand(lower_bound=workspace["y"][0], upper_bound=workspace["y"][1], shape=env_ids.shape)
        return (rand_x.unsqueeze(1), rand_y.unsqueeze(1))

    # For resetting specific environments. It resets joints, randomizes cube positions, and sets target goal poses for each cube
    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        
        # update object buffer once at start of reset
        self.objects.update(dt=self.physics_dt)

        for arm_name, arm_info in self.cfg.robot_arm_info_dict.items():
            # reset joint positions and velocities to zero
            initial_joints = torch.zeros((len(env_ids), self.cfg.arm_num_base_joints + 1), device=device)
            joint_vel = torch.zeros_like(initial_joints)
            self.arms[arm_name].write_joint_position_to_sim(initial_joints, self._joint_ids, env_ids)
            self.arms[arm_name].write_joint_velocity_to_sim(joint_vel, self._joint_ids, env_ids)
            self.arms[arm_name].set_joint_position_target(initial_joints, self._joint_ids, env_ids)
            self.arms[arm_name].write_data_to_sim()

            # reset cube pose with random x, y and fixed z, quat
            arm_target_name = arm_info["target"]
            arm_target_idx = self.objects.find_objects(arm_target_name)[0]
            state_w = self.objects.data.object_link_state_w[env_ids, arm_target_idx].unsqueeze(1)

            target_poses_w = torch.zeros((len(env_ids), 1, 7), device=device)
            target_poses_w[:, :, 2] = 0.835  # z on top of table
            target_poses_w[:, :, 3] = 1      # rotation as unit quaternion

            # sample random x, y for current position and goal
            target_poses_env = self.__state_w_to_state_env(target_poses_w, env_ids)
            goal_pose_env = target_poses_env.clone()
            target_poses_env[:, :, 0], target_poses_env[:, :, 1] = self._get_rand_point_in_workspace(arm_info["workspace"], env_ids)
            goal_pose_env[:, :, 0], goal_pose_env[:, :, 1] = self._get_rand_point_in_workspace(arm_info["workspace"], env_ids)

            # update internal goal tensor and set new cube position
            self._goal_pose_w[:, arm_target_idx, :] = self.__state_env_to_state_w(goal_pose_env, env_ids)
            target_poses_w = self.__state_env_to_state_w(target_poses_env, env_ids)

            obj_vel = torch.zeros((len(env_ids), 1, 6), device=device)
            self.objects.write_object_pose_to_sim(object_pose=target_poses_w, env_ids=env_ids, object_ids=arm_target_idx)
            self.objects.write_object_velocity_to_sim(object_velocity=obj_vel, env_ids=env_ids, object_ids=arm_target_idx)
