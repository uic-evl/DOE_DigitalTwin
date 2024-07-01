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
        self.sock = None  # Initialize the socket variable

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")
        if self.sock:
            self.sock.close()
        self.context.destroy()

    def on_play(self):
        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect('tcp://130.202.141.60:12344')
        self.sock.subscribe('')
        logger.warn("ZMQ socket Set")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False
        if self.sock:
            self.sock.close()

    def on_update(self, current_time: float, delta_time: float):
        try:
            message = self.sock.recv_multipart(flags=zmq.NOBLOCK)
            #logger.warn(message[0])

            #if len(message) != 4:
            #    logger.error(f"Unexpected message format: {message}")
            #    return

            position_data = message[1].decode('utf-8').replace("(", "").replace(")", "")
            rotation_data = message[3].decode('utf-8').replace("(", "").replace(")", "")

            position_data = position_data.split(",")
            rotation_data = rotation_data.split(",")

            #if len(position_data) != 3 or len(rotation_data) != 3:
            #    logger.error(f"Unexpected message format: {message}")
            #    return

            x = float(position_data[0]) * -1.0
            y = float(position_data[1])
            z = float(position_data[2]) * -1.0

            #logger.warn(rotation_data)
    
            r = float(rotation_data[0])
            i = float(rotation_data[1])
            j = float(rotation_data[2])
            k = float(rotation_data[3])

            physicsUtils.set_or_add_translate_op(self.curr_prim, (z,x,y))
            physicsUtils.set_or_add_orient_op(self.curr_prim, Gf.Quatf(r,i,j,k))

            # Update transform using UsdGeom.XformCommonAPI
            #xformable = UsdGeom.XformCommonAPI(self.curr_prim)
            #xformable.SetTranslate((z, x, y))
            #xformable.SetRotate((rot_x, rot_y, rot_z))

            # Print statements to confirm data
            #print(f"Updated Position: {(z, x, y)}")
            #print(f"Updated Rotation: {(rot_x, rot_y, rot_z)}")

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass  # no message was ready (yet!)
            else:
                logger.error(f"ZMQ Error: {str(e)}")