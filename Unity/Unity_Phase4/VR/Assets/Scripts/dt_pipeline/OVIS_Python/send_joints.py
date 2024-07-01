import logging
import math
from pprint import pprint

import numpy as np
import omni
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.scripting import BehaviorScript
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)

try:
    import zmq
except ModuleNotFoundError:
    omni.kit.pipapi.install("zmq")
    import zmq

logger = logging.getLogger(__name__)
commands = []

class RobotControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

        self.robot = None

        self.context = zmq.Context()

        self.sock = self.context.socket(zmq.PUB)
        #self.sock.connect('tcp://130.202.141.68:5560') #FOR ER LINK
        self.sock.bind("tcp://*:12346") #FOR UNITY

        #socket to pi in rpl tcp://146.137.240.73:5560
        # defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        # omni.usd.get_context().get_stage()

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

        self.robot = Articulation(str(self.prim_path))
        self.robot.initialize()

    def on_update(self, current_time: float, delta_time: float):
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        
        # Get joint positions from the robot
        joints = self.robot.get_joint_positions()
        
        # Convert joint positions to strings and then to a single string
        joints_str = str([str(j) for j in joints])
        
        # Send joint positions as a single string
        try:
            self.sock.send_string(joints_str)
        except zmq.ZMQError as exc:
            logger.warn(exc)