import logging
import math
from pprint import pprint
import time
from datetime import datetime
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

logger = logging.getLogger(__name__)
commands = []

class CustomLogger(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.frameNumber = 0

        # Get cmd message attribute
        self.cmd_message = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("cmd_message")

        # Get virtaul endpoint position 
        self.v_endpoint = self.stage.GetPrimAtPath("/World/firefighter/joint6_flange/pointTest").GetAttribute("endpoint")

        # Get IK Status
        self.ik_status = self.stage.GetPrimAtPath("/World/firefighter").GetAttribute("ik_status")


    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
       # Open file stream
        self.filestream = open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Recordings/v_arm.txt", "w", encoding="utf-8")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False

        # Close file stream
        self.filestream.close()


    def on_update(self, current_time: float, delta_time: float):
        #logger.warn("hello from frame " + str(self.frameNumber))

        current_time = datetime.now()

        # Add time and frame number to data string 
        #data = str(current_time) + "," + str(self.frameNumber) + ","
        data = str(self.frameNumber) + ","

        # Add cmd message to data string
        data = data + self.cmd_message.Get()

        # Add virtual endpoint position to data string
        #data = data + str(self.v_endpoint.Get())

        # Add IK status to data string
        data = ";" + str(self.ik_status.Get())
        data = data + "\n"

        # Write data string to file
        self.filestream.writelines(data) 

        self.frameNumber = self.frameNumber + 1