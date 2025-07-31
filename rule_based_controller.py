import argparse
import math
from isaaclab.app import AppLauncher
import time
import threading
import torch
import numpy as np 
import torch.multiprocessing as mp 
from itertools import islice
import pytorch_kinematics as pk
from isaaclab.utils.math import subtract_frame_transforms
import json

# Choose device based on gpu availability
device = "cuda" if torch.cuda.is_available() else "cpu"
dtype = torch.float64

# Main controller class for computing arm joint positions using ik
class Arms_Controller:
    def __init__(self, init_obs, init_target_joints, num_envs, arm_info_dict):
        self._num_envs = num_envs
        self._arm_names = list(arm_info_dict.keys())
        self._num_arms = len(self._arm_names)
        self._arm_info_dict = arm_info_dict

        # each arm has a fixed target orientation
        self._target_orientation = torch.tensor([[[0,1,0,0], [0,1,0,0]] for _ in range(self._num_envs)], dtype=torch.float32, device=device)

        # flags for when to update the stage for each arm
        self._stages_updated = torch.tensor([[True for _ in range(self._num_arms)] for _ in range(self._num_envs)], device=device)

        # holds current joint and position targets
        self._target_joints = init_target_joints
        self._target_positions = torch.empty((self._num_envs, self._num_arms, 3), device=device)

        # current stage and convergence flags for each arm
        self._stages = torch.ones((num_envs, self._num_arms), device=device)
        self._converged = torch.full((num_envs, self._num_arms), False, dtype=torch.bool, device=device)

        # initialize solver and arm pose
        self._init_solver()
        self._get_arm_base_state(init_obs)

        self._update_stages = False

    # Loads robot model and builds the kinematic chain for ik
    def _create_chain_from_urdf(self):
        urdf_path = r"C:\Omniverse_Files\isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.windows-x86_64.release\standalone_examples\tutorials\myarm_300_pi.urdf"
        chain = pk.build_serial_chain_from_urdf(open(urdf_path, mode="rb").read(), "joint7")
        chain.to(device=device)
        self._chain = chain
        return chain

    # Extracts the robot base pose from initial observations
    def _get_arm_base_state(self, obs):
        arm_state = torch.stack(tuple(obs["arms"][arm_name]["root_pose_w"].squeeze() for arm_name in self._arm_names), dim=1)
        self._arm_base_translation, self._arm_base_orientation = arm_state.view(self._num_envs*self._num_arms, -1)[:,:3], arm_state.view(self._num_envs*self._num_arms, -1)[:,3:7]

    # Returns end effector position and orientation in the robot's base frame
    def _get_ee_pose(self, obs):
        ee_pose_w = torch.stack(tuple(obs["arms"][arm_name]["ee_pose_w"].squeeze() for arm_name in self._arm_names), dim=1).view(self._num_envs*self._num_arms, -1)
        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            self._arm_base_translation, self._arm_base_orientation, ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        return ee_pos_b, ee_quat_b 
    
    # Reinitialize robot base frame (used on reset)
    def _init_robot_frame(self, init_obs):
        self._get_arm_base_state(init_obs)
    
    # Initialize inverse kinematics solver
    def _init_solver(self):
        chain = self._create_chain_from_urdf()
        lim = torch.tensor(chain.get_joint_limits(), device=device)
        self._ik = pk.PseudoInverseIK(
            chain, 
            max_iterations=30,
            num_retries=10,
            early_stopping_no_improvement_patience=5,
            joint_limits=lim.T,
            early_stopping_any_converged=True,
            early_stopping_no_improvement="all",
            debug=False,
            lr=0.2
        )

    # Orders cube and goal positions based on which arm is targeting them
    def _get_ordered_cube_positions(self, obs):
        cube_positions = obs["objects"]["root_pose_w"][:,:,0:3].squeeze()
        goal_positions = obs["objects"]["goal_pose_w"][:,:,0:3].squeeze()
        ordered_cube_positions = torch.empty(cube_positions.shape, device=device)
        ordered_goal_positions = torch.empty(cube_positions.shape, device=device)

        for arm_name, arm_index in zip(self._arm_names, range(self._num_arms)):
            arm_target_name = self._arm_info_dict[arm_name]["target"]
            target_index = obs["objects"]["idxs"][arm_target_name]
            ordered_cube_positions[:, arm_index, :] = cube_positions[:, target_index, :]
            ordered_goal_positions[:, arm_index, :] = goal_positions[:, target_index, :]

        return ordered_cube_positions, ordered_goal_positions

    # Updates target position based on current stage
    def _update_target_positions(self, obs):
        arm_cube_targets, goal_positions = self._get_ordered_cube_positions(obs)

        # each block updates targets only for the relevant stage
        env_idx, arm_idx = torch.where(self._stages_updated & (self._stages==1))
        if env_idx.numel():
            self._target_positions[env_idx, arm_idx] = arm_cube_targets[env_idx, arm_idx] + torch.tensor([0, 0, 0.125], device=device).unsqueeze(0)

        env_idx, arm_idx = torch.where(self._stages_updated & (self._stages==2))
        if env_idx.numel():
            self._target_positions[env_idx, arm_idx] = arm_cube_targets[env_idx, arm_idx] + torch.tensor([0, 0, 0.123], device=device).unsqueeze(0)

        env_idx, arm_idx = torch.where(self._stages_updated & (self._stages==4))
        if env_idx.numel():
            self._target_positions[env_idx, arm_idx] = arm_cube_targets[env_idx, arm_idx] + torch.tensor([0, 0, 0.126], device=device).unsqueeze(0)

        env_idx, arm_idx = torch.where(self._stages_updated & (self._stages==5))
        if env_idx.numel():
            self._target_positions[env_idx, arm_idx] = goal_positions[env_idx, arm_idx] + torch.tensor([0, 0, 0.125], device=device).unsqueeze(0)

        env_idx, arm_idx = torch.where(self._stages_updated & (self._stages==6))
        if env_idx.numel():
            self._target_positions[env_idx, arm_idx] = goal_positions[env_idx, arm_idx] + torch.tensor([0, 0, 0.123], device=device).unsqueeze(0)

    # Checks joint error and updates stage when close to target
    def _update_stage(self, curr_joint_pos, dones=None, thresh=0.01):
        gripper_error = torch.linalg.vector_norm((curr_joint_pos[:, :, 10] - self._target_joints[:, :, 10]).view(-1, 1), dim=1)
        base_joints_error = torch.linalg.vector_norm((curr_joint_pos[:, :, :7] - self._target_joints[:, :, :7]).view(-1, 7), dim=1)
        joint_error = (gripper_error + base_joints_error).view(self._num_envs, self._num_arms)

        dones = dones.view(-1, 1)
        dones = torch.cat((dones, dones), dim=1)

        self._stages_updated = ((joint_error < thresh) & (self._stages <= 6)) | dones
        self._stages[self._stages_updated] += 1
        self._stages[dones] = 1

    # Makes sure joint angles stay between -π and π
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

# Helper: calculates the next joint targets using inverse kinematics
def get_joints(self, obs, dones=None):
    # get the current joint positions for all arms across all envs
    curr_joint_pos = torch.stack(tuple(obs["arms"][arm_name]["joint_pos"][:, :] for arm_name in self._arm_names), dim=1)
    
    # if stage tracking is enabled, check if arms are ready to move to the next stage
    if self._update_stages: 
        self._update_stage(curr_joint_pos, dones)
    self._update_stages = True 

    # find the env/arm pairs that need to update their targets
    updated_env_idx, updated_arm_idx = torch.where((self._stages_updated) | (~self._converged))

    # if no arms need updating, just return the last target joints
    if not updated_env_idx.numel():
        return self._target_joints

    # update the xyz targets based on the arm’s current stage
    self._update_target_positions(obs)

    # flatten the 2d env/arm indices into 1d for indexing batched tensors
    idxs = updated_env_idx * self._num_arms + updated_arm_idx

    # build 7d goal poses (xyz + quat) for each end effector
    ee_goals = torch.cat((
        self._target_positions.view(self._num_envs * self._num_arms, -1),
        self._target_orientation.view(self._num_envs * self._num_arms, -1)
    ), dim=1).view(-1, 7)

    # convert world frame ee goals into robot base frame
    ee_goals_b = torch.cat(subtract_frame_transforms(
        self._arm_base_translation, self._arm_base_orientation, ee_goals[:, :3], ee_goals[:, 3:7]
    ), dim=1)

    # set warm start guesses for ik solver using current joint positions
    self._ik.retry_configs = curr_joint_pos[updated_env_idx, updated_arm_idx, :7]

    # solve inverse kinematics to get best joint angles
    sol = self._ik.solve(pk.Transform3d(pos=ee_goals_b[idxs, :3], rot=ee_goals_b[idxs, 3:7], device=device))
    best_sols = sol.best_err.argmin(1)

    # pick the best joint solution for each environment-arm pair
    batch_idxs = torch.arange(updated_env_idx.numel(), device=device)
    base_joints = sol.best_q[batch_idxs, best_sols].view(-1, 7)

    # update convergence flags for arms that successfully solved ik
    self._converged[updated_env_idx, updated_arm_idx] = sol.converged[batch_idxs, best_sols]

    # replace any nan values with 0 to avoid corrupting the sim
    base_joints[torch.isnan(base_joints)] = 0

    # wrap angles to keep them between -π and π
    base_joints = self.wrap_to_pi(base_joints)

    # compute gripper action based on stage (gripper closes between stages 3 and 6)
    gripper_on = ((self._stages[updated_env_idx, updated_arm_idx] > 2) & (self._stages[updated_env_idx, updated_arm_idx] < 7))
    gripper = torch.zeros(gripper_on.shape, device=device)
    gripper[gripper_on] = -0.5
    gripper = gripper.unsqueeze(1)  # match shape with base_joints

    # update the full joint targets for base and gripper
    self._target_joints[updated_env_idx, updated_arm_idx, :7] = base_joints
    self._target_joints[updated_env_idx, updated_arm_idx, 7:] = gripper

    # return the updated target joints to be sent to the sim
    return self._target_joints


# Entry point for running the script
if __name__ == '__main__':
    # set up cli args and launch app
    parser = argparse.ArgumentParser(description="Hotcell enviornment test.")
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    # launch the simulation app using the parsed cli args
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app


    from hotcell_env import HotCellEnvCfg
    cfg = HotCellEnvCfg()

    from hotcell_env import HotCellEnv
    hot_cell_env = HotCellEnv(cfg)
        
    # get the simulation timestep
    dt = hot_cell_env.step_dt

    # reset the environment and get the initial observation
    obs = hot_cell_env.reset()[0]

    # get number of arms and environments
    num_arms = len(hot_cell_env.arm_names)
    num_envs = cfg.num_envs

    # initialize dones tensor to track episode completion
    dones = torch.tensor([[False]*num_arms for _ in range(num_envs)])

    # initialize joint targets (13 = 7 base joints + 6 gripper components per arm)
    target_joints = torch.zeros(num_envs, 2, 13, device=device)

    # set initial gripper joint to -1 (closed)
    target_joints[:, :, 0] = -1

    # initialize the dual-arm controller with current observations and targets
    arm_controller = Arms_Controller(init_obs=obs, init_target_joints=target_joints, num_envs=num_envs, arm_info_dict=cfg.robot_arm_info_dict)

    # pause before first step to allow initialization to settle
    time.sleep(dt)

    # Main simulation loop
    while simulation_app.is_running():
        start_time = time.time()

        # use controller to calculate new joint targets based on current state
        joints = arm_controller.get_joints(obs, dones)

        # apply new joint targets and step the simulation forward
        obs, rewards, truncated, terminated, _ = hot_cell_env.step(joints)
        print("Obs: ", obs)

        # update dones tensor to reflect current environment states
        dones = truncated | terminated

        # sleep the remaining time to maintain real-time stepping
        elapsed_time = time.time() - start_time
        if elapsed_time < dt:
            time.sleep(dt - elapsed_time)
