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

        # Set starting point 
        self.study_area = stage.GetPrimAtPath("/World/StudyArea")
        pose = omni.usd.get_world_transform_matrix(self.study_area)
        trans = pose.ExtractTranslation()

        #Translate the origin from cube center to corner
        sx = trans[0] + 0.05 
        sy = trans[1] + 0.05 
        sz = trans[2] + 0.025

        self.starting_point = Gf.Vec3f(sx,sy,sz)

        self.x_max = sx + 0.1
        self.y_max = sy + 0.1
        self.z_max = sz + 0.05

        # Fixed Update Loop Utils
        self.time_stamp = 0.0
        self.time_step = 0
        self.time_delay = 3 

        # Get path points from file
        with open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Output/output.txt") as f:
            self.lines = f.read().splitlines()

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")


    def on_play(self):
        self.waiting = False
        self.time_step = 0

        self.filestream = open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Output/distance.txt", "w", encoding="utf-8")

        pass

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False
         
        # Close file stream
        self.filestream.close()


    def on_update(self, current_time: float, delta_time: float):
        '''
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        trans = pose.ExtractTranslation()

        x = trans[0] + 0.0005
        y = trans[1] + 0.0005
        z = trans[2] + 0.0005
        '''

        # Every tenth of a second, move to the next point
        if self.waiting is False and self.time_step <= len(self.lines):
            # Write distance measurement BEFORE new command is sent
            data = self.lines[self.time_step] + ";" + str(self.public_distance.Get()) + "\n"
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

        
        if current_time > (self.time_stamp + self.time_delay):
            self.waiting = False 

        # Apply translation to Target
        #physicsUtils.set_or_add_translate_op(self.curr_prim, (x,y,z))

