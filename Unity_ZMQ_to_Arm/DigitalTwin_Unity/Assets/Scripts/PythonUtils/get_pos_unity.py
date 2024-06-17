import logging
import math
from pprint import pprint
import time

import typing
import omni.usd
from pxr import Usd, Gf

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

#Get self (Target Cube) Position

class TargetControl(BehaviorScript):

    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        
        dc = _dynamic_control.acquire_dynamic_control_interface()

        # Testing initial pos extraction
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        trans = pose.ExtractTranslation()

        logger.warn(trans)

        self.context = zmq.Context()

        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect('tcp://localhost:12344')
        self.sock.subscribe('')

        logger.warn("ZMQ socket Set")

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        self.sock.close()
        self.context.destroy()

    def on_play(self):
        logger.warn("Hello from on_play()")

    def on_update(self, current_time: float, delta_time: float):
        try:
            logger.warn("Getting from Unity")
            message = self.sock.recv_string()

            # Process string
            logger.warn(message)
            message = message[1:-1]
            xform_data = message.split(",")

            x = float(xform_data[0])
            y = float(xform_data[1])
            z = float(xform_data[2])

            # Update xform
            physicsUtils.set_or_add_translate_op(self.curr_prim, (x,z,y))

        except ValueError as e:
            print("Message Recv Failed")


