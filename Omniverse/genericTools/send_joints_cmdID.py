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

# This script should be placed on the robot xform in the OVIS Scene.
# It gets the joints and sends to Unity

class RobotControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

        self.robot = None

        # ZMQ Networking for Joint Publisher
        self.joint_port = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint_port").Get()

        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.PUB)
        self.sock.bind(f"tcp://*:{self.joint_port}")

        self.had_first_update = False

        # Make cmd message public 
        self.cmd_message = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("cmd_message")

        # Initialize cmdID
        self.cmdID = 0

        # Fixed Update Loop Utils
        self.time_stamp = 0.0
        self.time_step = 0
        self.time_delay = 0.5 # Large delay to account for errors

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        self.context.destroy()

    def on_play(self):
        logger.info(f"{__class__.__name__}.on_play()->{self.prim_path}")
        self.had_first_update = False

        self.waiting = False
        self.cmdID = 0

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

        # Ignore gripper joints (for now)
        #logger.warn(self.robot.dof_names)
        joints = joints[:6]
        
        # Convert joint positions to strings and then to a single string
        joints_str = str([str(j) for j in joints])

        # Only send every so many times a second
        if self.waiting is False:
            self.waiting = True
            self.time_stamp = current_time
            self.time_step = self.time_step + 1

            # Prepare command
            self.cmd_message.Set(str(self.cmdID)+ "," + joints_str)
            self.cmdID = self.cmdID + 1

            #logger.warn(self.cmd_message.Get())

            # Send joint positions as a single string
            try:
                self.sock.send_string(self.cmd_message.Get())
            except zmq.ZMQError as exc:
                logger.warn(exc)

        if current_time > (self.time_stamp + self.time_delay):
            self.waiting = False 