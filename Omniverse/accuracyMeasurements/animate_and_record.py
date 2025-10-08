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

# This script should be placed on the IK Target in the OVIS Scene.
# It will animate the Target through a defined volume
# It will also print a log to be used in a synchronized playback

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
        #logger.warn(trans)

        # Get distance attribute
        # TO DO: Don't hardcode this
        self.public_distance = self.stage.GetPrimAtPath("/World/firefighter").GetAttribute("public_distance")

        # Get ik attribute
        # TO DO: Don't hardcode this
        self.ik_status = self.stage.GetPrimAtPath("/World/firefighter").GetAttribute("ik_status")

        # Get cmd message attribute
        self.cmd_message = self.stage.GetPrimAtPath("/World/firefighter").GetAttribute("cmd_message")

        # Fixed Update Loop Utils
        self.time_stamp = 0.0
        self.time_step = 0
        self.time_delay = 1 # Large delay to account for errors

        # Get path points from file
        with open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Output/output.txt") as f:
            self.lines = f.read().splitlines()

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")


    def on_play(self):
        self.waiting = False
        self.time_step = 0

        self.filestream = open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Recordings/anim_recording_varm.txt", "w", encoding="utf-8")

        pass

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False
         
        # Close file stream
        self.filestream.close()


    def on_update(self, current_time: float, delta_time: float):
        # Get lines
        printLine = str(self.time_step + 1) + "/" + str(len(self.lines))
        logger.warn(printLine)

        # Every tenth of a second, move to the next point
        if self.waiting is False and self.time_step < len(self.lines) - 1:
            # Write log BEFORE new command is sent
            # timestep;IKTargetPoint;cmd_message;ik_status
            data = str(self.time_step) + self.lines[self.time_step] + ";" + str(self.cmd_message.Get()) + ";" + str(self.ik_status.Get())+ "\n"
            self.filestream.writelines(data) 

            self.waiting = True
            self.time_stamp = current_time
            self.time_step = self.time_step + 1

            #The work
            #logger.warn(self.time_step)

            data = self.lines[self.time_step] 
            data = data.replace("(","")
            data = data.replace(")","")
            point = data.split(",")

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            physicsUtils.set_or_add_translate_op(self.curr_prim, (x,y,z))
        
        if(self.time_step >= len(self.lines)):
            logger.warn("Animation complete")
        
        if current_time > (self.time_stamp + self.time_delay):
            self.waiting = False 

