import logging
import math
from pprint import pprint
import time
import typing
import omni.usd
from pxr import Usd, Gf, UsdGeom
import numpy as np
import omni
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
from omni.physx.scripts import physicsUtils
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.scripting import BehaviorScript
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)
import zmq


logger = logging.getLogger(__name__)
commands = []

# This script should be placed on the robot xform in the OVIS Scene.
# It gets the joints and sends to Unity

# Source: 
# https://github.com/mati-nvidia/developer-office-hours/blob/main/exts/maticodes.doh_2023_04_07/scripts/get_local_transform.py
def decompose_matrix(mat: Gf.Matrix4d):
    reversed_ident_mtx = reversed(Gf.Matrix3d())

    translate = mat.ExtractTranslation()
    scale = Gf.Vec3d(*(v.GetLength() for v in mat.ExtractRotationMatrix()))
    #must remove scaling from mtx before calculating rotations
    mat.Orthonormalize()
    #without reversed this seems to return angles in ZYX order
    rotate = Gf.Vec3d(*reversed(mat.ExtractRotation().Decompose(*reversed_ident_mtx)))
    return translate, rotate, scale

class SendActorXform(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)

        # ZMQ Networking for Joint Publisher
        self.xform_port = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("xform_port").Get()

        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.PUB)
        self.sock.bind(f"tcp://*:{self.xform_port}")

        self.had_first_update = False

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        self.context.destroy()

    def on_play(self):
        logger.info(f"{__class__.__name__}.on_play()->{self.prim_path}")

        #logger.warn("UR Playing")

        self.had_first_update = False

    def on_pause(self):
        logger.info(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")

        self.had_first_update = False

    def on_first_update(self, current_time: float, delta_time: float):
        self.had_first_update = True


    def on_update(self, current_time: float, delta_time: float):
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        xform = decompose_matrix(pose)
        trans = pose.ExtractTranslation()
        rot = xform[1]

        #logger.warn(pose)
        #logger.warn(trans)
        #logger.warn(rot)
        
        x = trans[0]
        y = trans[1]
        z = trans[2]

        rx = rot[0]
        ry = rot[1]
        rz = rot[2]

        msg_str = str(x) + "," + str(y) + "," + str(z) + "," +  str(rx) + "," + str(ry) + "," + str(rz)
        #logger.warn(msg_str)

        # Send joint positions as a single string
        try:
            self.sock.send_string(msg_str)
        except zmq.ZMQError as exc:
            logger.warn(exc)