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


# MOTION_GEN_ALGO = 'RMPFlow'
MOTION_GEN_ALGO = 'IK'
PATH_BASE = Path("C:\\Users\\halle\\Documents\\DigitalTwin\\DOE_DigitalTwin\\")
logger = logging.getLogger(__name__)

class InspectorVariable(property):
    def __init__(self, name: str, value_type: Sdf.ValueTypeName, default=None) -> None:
        def get_attr(owner):
            assert isinstance(owner, BehaviorScript),\
                'InspectorVariable should only be attached to an instance of BehaviorScript'
            attr = owner.prim.GetAttribute(name)
            if not attr:
                attr = owner.prim.CreateAttribute(name, value_type)
                if default is not None:
                    attr.Set(default)

            return attr

        def fget(owner):
            return get_attr(owner).Get()

        def fset(owner, value):
            get_attr(owner).Set(value)

        super().__init__(fget, fset)


class RobotControl(BehaviorScript):
    shader = InspectorVariable('moo', Sdf.ValueTypeNames.String, '')
    
    
    def __init__(self, prim_path: Sdf.Path):
        super().__init__(prim_path)

        # Make InspectorVariables populate in the Raw USD Properties for editing
        for k, v in self.__class__.__dict__.items():
            if isinstance(v, InspectorVariable):
                v.__get__(self)
                
    def on_init(self):
        self.robot = None
        self.follow_target = None
        self.had_first_update = False


        self.ee_name = 'joint5'
        self.ee_prim = self.stage.GetPrimAtPath(str(self.prim_path) + '/' + self.ee_name)
        logger.error("---ee_name: "+ str(self.ee_name) + " ee_prim: " + str(self.ee_prim))

        if MOTION_GEN_ALGO == 'RMPFlow':
            self.motion_gen_algo = RmpFlow(
                urdf_path=str(PATH_BASE / 'robot_data/ur5e.urdf'), 
                rmpflow_config_path=str(PATH_BASE / 'robot_data/ur_rmpflow.yaml'), 
                robot_description_path=str(PATH_BASE / 'robot_data/ur_description.yaml'),
                end_effector_frame_name=self.ee_name,
                maximum_substep_size=.0034,
            )
            logger.warn(f'RMPFlow loaded')
        elif MOTION_GEN_ALGO == 'IK':
            #logger.warn(str(PATH_BASE) + '/robot_data/myarm_300_pi/myarm_300_pi.urdf')
            self.motion_gen_algo = LulaKinematicsSolver(
                robot_description_path=str(PATH_BASE) + '/Robot_Files/ER_MyArm300/description/rd_myarm.yaml',
                urdf_path=str(PATH_BASE) + "/Robot_Files/ER_MyArm300/description/myarm_300_pi.urdf",
            )
            logger.warn('IK loaded')
        else:
            logger.warn(f'BAD MOTION_GEN_ALGO {MOTION_GEN_ALGO}')

        logger.warn(f'This is the path base: {PATH_BASE}')
        logger.warn(f'Finished initializing')

    def on_destroy(self):
        pass

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
        #get the articulation root from the prim I am attatched to. must be attatched to the articulation root
        self.robot.initialize()

        if MOTION_GEN_ALGO == 'RMPFlow':
            self.motion_gen_solver = ArticulationMotionPolicy(self.robot, self.motion_gen_algo, default_physics_dt=1/120)
        elif MOTION_GEN_ALGO == 'IK':
            self.motion_gen_solver = ArticulationKinematicsSolver(self.robot, self.motion_gen_algo, self.ee_name)

    def on_update(self, current_time: float, delta_time: float):
        # Do any initialization that couldn't be done in on_play()
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
            logger.error("following Target @ path: " + str(self._prim_path))
            #self.ee_prim = self.stage.GetPrimAtPath(str(self.prim_path) + '/' + self.ee_name)
        follow_target = self.stage.GetPrimAtPath(str(self.prim_path) + '/Target')

        rot = get_world_rotation(follow_target)
        direction = rot.TransformDir(Gf.Vec3d(0, 0, 1))
        w = rot.GetQuat().GetReal()
        im = rot.GetQuat().GetImaginary()
        rot = np.array((w, im[0], im[1], im[2]))

        pos = get_world_translation(follow_target)
        # pos -= self.ee_offset * direction
        pos = np.array(pos)

        target = (pos, rot)

        robot_pose = self.robot.get_world_pose() # this is defined by omni not rory
        self.motion_gen_algo.set_robot_base_pose(robot_pose[0], robot_pose[1])

        #logger.warn(f'Pos: {robot_pose[0]}')
        #logger.warn(f'Rot: {robot_pose[1]}')


        if MOTION_GEN_ALGO == 'RMPFlow':
            self.motion_gen_algo.set_end_effector_target(
                target_position=target[0],
                target_orientation=target[1],
            )
            action = self.motion_gen_solver.get_next_articulation_action()
            self.robot.apply_action(action)
        elif MOTION_GEN_ALGO == 'IK':
            action, success = self.motion_gen_solver.compute_inverse_kinematics(
                target_position=target[0],
                target_orientation=target[1],
            )
            if success:
                self.robot.apply_action(action)
                #prim_color = self.stage.GetPrimAtPath(self.shader)
                #attr_color = prim_color.GetAttribute('inputs:diffuse_tint')
                #attr_color.Set(Gf.Vec3f(0.5,0.5,0.5))
            else:
                #self.robot.apply_action(action)
                logger.warn(f'{self.prim_path} - IK failed')
                #prim_color = self.stage.GetPrimAtPath(self.shader)
                #attr_color = prim_color.GetAttribute('inputs:diffuse_tint')
                #attr_color.Set(Gf.Vec3f(2,0,0))
                #logger.error('The target is RED because it is too far from the UR5e. Please move it back to bounds.')

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


