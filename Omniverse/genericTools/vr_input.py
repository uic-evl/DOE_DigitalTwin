import logging
import math
from pathlib import Path
from pprint import pprint
import time
from typing import Tuple
import os

import omni.kit.commands

import numpy as np
import omni
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
import omni.usd.commands
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.motion_generation import (ArticulationKinematicsSolver,
                                          ArticulationMotionPolicy,
                                          LulaKinematicsSolver, RmpFlow)
from omni.kit.scripting import BehaviorScript
from omni.physx import get_physx_scene_query_interface
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)

try:
    import zmq
except ModuleNotFoundError:
    omni.kit.pipapi.install("zmq")
    import zmq


MOTION_GEN_ALGO = 'IK'
logger = logging.getLogger(__name__)

# Setup Instructions:
#
# This file should be attached to the parent prim of your robot arm.
# 
# 1. Add the following Attributes to that prim (attribute name, attribute type):
#   * URDF, Asset
#   * robot_description, Asset
#   * ee_name, string
#   * IKport, int
#   * joint_port, int
#
# 2. Fill the newly added attributes with the correct information
#
# 3. Create the "Target" as a child of the robot prim
#   * If you will be controlling the Target through Omniverse VR, "Target" should be a Cube prim
#   * Otherwise, "Target" should be an empty prim with a Cube as the child.
#
# 4. Create the "Target_Mat" OmniPBR material
#   * Assign "Target_Mat" to the Target Cube
#   * Add the following attributes to "Target_Mat":
#      * ik_good, Color3f
#      * ik_bad, Color3f
#   * Then select the colors you want to see in the Properties panel

# Port conventions
# IKCheck: 12347
# Joint Publisher: 12346
# Target Publisher: 12344

class RobotControl(BehaviorScript):
    def __init__(self, prim_path: Sdf.Path):
        super().__init__(prim_path)

                
    def on_init(self):
        self.robot = None
        self.follow_target = None
        self.had_first_update = False

        # ZMQ Networking for failed IK
        self.IKport = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("IKport").Get()

        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.PUB)
        self.sock.bind(f"tcp://*:{self.IKport}")

        # Get end effector name from asset
        self.ee_name = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("ee_name").Get()
        self.ee_prim = self.stage.GetPrimAtPath(str(self.prim_path) + '/' + self.ee_name)

        # Get URDF file from asset
        self.urdf_file = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("URDF")
        self.urdf_file_name = str(self.urdf_file.Get())
        self.urdf_file_name = self.urdf_file_name.replace("@", "")

        # Get Robot Description YAML from asset
        self.description_file = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("robot_description")
        self.description_file_name = str(self.description_file.Get())
        self.description_file_name = self.description_file_name.replace("@", "")

        # Get Target Material and colors
        # This assumes a Target Material at the below path,
        # and assumes "ik_good" and "ik_bad" attributes have been added
        self.mat_name = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("mat_name").Get()

        self.target_mat = self.stage.GetPrimAtPath("/World/Looks/" + str(self.mat_name) + "/Shader").GetAttribute("inputs:diffuse_color_constant")
        self.ik_good = self.stage.GetPrimAtPath("/World/Looks/" + str(self.mat_name)).GetAttribute("ik_good")
        self.ik_bad = self.stage.GetPrimAtPath("/World/Looks/" + str(self.mat_name)).GetAttribute("ik_bad")

        if MOTION_GEN_ALGO == 'IK':
            self.motion_gen_algo = LulaKinematicsSolver(
                robot_description_path=str(self.description_file_name),
                urdf_path=str(self.urdf_file_name),
            )
            logger.warn('IK loaded')
        else:
            logger.warn(f'BAD MOTION_GEN_ALGO {MOTION_GEN_ALGO}')

        logger.warn(f'Finished initializing')

    def on_destroy(self):
        self.context.destroy()

    def on_play(self):
        logger.warn(f'on_play')
        self.had_first_update = False

    def on_pause(self):
        pass

    def on_stop(self):
        self.had_first_update = False

    def on_first_update(self, current_time: float, delta_time: float):
        self.had_first_update = True

        self.robot = Articulation(str(self.prim_path))
        self.robot.initialize()

        if MOTION_GEN_ALGO == 'IK':
            self.motion_gen_solver = ArticulationKinematicsSolver(self.robot, self.motion_gen_algo, self.ee_name)

    def on_update(self, current_time: float, delta_time: float):
        # Do any initialization that couldn't be done in on_play()
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        follow_target = self.stage.GetPrimAtPath(str(self.prim_path) + '/Target')

        rot = get_world_rotation(follow_target)
        direction = rot.TransformDir(Gf.Vec3d(0, 0, 1))
        w = rot.GetQuat().GetReal()
        im = rot.GetQuat().GetImaginary()
        rot = np.array((w, im[0], im[1], im[2]))

        pos = get_world_translation(follow_target)
        pos = np.array(pos)

        target = (pos, rot)

        robot_pose = self.robot.get_world_pose() 
        self.motion_gen_algo.set_robot_base_pose(robot_pose[0], robot_pose[1])

        if MOTION_GEN_ALGO == 'IK':
            action, success = self.motion_gen_solver.compute_inverse_kinematics(
                target_position=target[0],
                target_orientation=target[1],
            )
            if success:
                self.robot.apply_action(action)
                self.target_mat.Set(self.ik_good.Get())
                try:
                    self.sock.send_string("0")
                except zmq.ZMQError as exc:
                    logger.warn(exc)

            else:
                #logger.warn(f'{self.prim_path} - IK failed')
                self.target_mat.Set(self.ik_bad.Get())
                try:
                    self.sock.send_string("1")
                except zmq.ZMQError as exc:
                    logger.warn(exc)


def get_world_pose(prim: Usd.Prim) -> Tuple[np.ndarray, np.ndarray]:
    pos = get_world_translation(prim)
    pos = np.array(pos)
    logger.warn("pos of: " + prim + "\n")

    rot = get_world_rotation(prim)
    w = rot.GetQuat().GetReal()
    im = rot.GetQuat().GetImaginary()
    rot = np.array((w, im[0], im[1], im[2]))

    return pos, rot

def get_world_translation(prim: Usd.Prim) -> Gf.Vec3d:
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    #logger.warn("translation of: " + prim + "\n")
    return translation

def get_world_rotation(prim: Usd.Prim) -> Gf.Rotation:
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    #logger.warn("rotation of: " + prim + "\n")
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return rotation 

def get_world_scale(prim: Usd.Prim) -> Gf.Vec3d:
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    logger.warn("scale of: " + prim + "\n")
    return scale


