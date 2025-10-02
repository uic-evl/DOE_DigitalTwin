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

        # Set starting point 
        self.study_area = stage.GetPrimAtPath("/World/StudyArea")
        pose = omni.usd.get_world_transform_matrix(self.study_area)
        trans = pose.ExtractTranslation()

        #Translate the origin from cube center to corner
        self.sx = trans[0] + 0.05 
        self.sy = trans[1] + 0.05 
        self.sz = trans[2] + 0.025

        #self.starting_point = Gf.Vec3f(sx,sy,sz)

        self.step_size = 0.0005

        self.x_max = self.sx + 0.1
        self.y_max = self.sy + 0.1
        self.z_max = self.sz + 0.05

        # Open file stream
        #self.filestream = open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Output/output.txt", "w", encoding="utf-8")

        # Create and fill path points
        self.path = []
        
        # Starting from the top corner
        # Up-down loop from top to bottom
        reverse_x = False
        for z_ind in range (0,11):
            logger.warn(z_ind)
            # Back-Forth loop
            for y_ind in range(0,21):
                logger.warn(y_ind)
                min = 0
                max = 100
                if reverse_x is False:
                    min = 0
                    max = 21
                else:
                    min = 21
                    max = 0
                # Left-Right loop
                for x_ind in range(min, max):

                    z = self.sz - (self.step_size * z_ind)
                    y = self.sy - (self.step_size * y_ind)
                    x = self.sx - (self.step_size * x_ind)
                    
                    logger.warn(x_ind)
                    self.path.append((x,y,z))
                    reverse_x = not reverse_x

         
    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")


    def on_play(self):
        pass
                    

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False


    def on_update(self, current_time: float, delta_time: float):
        pass

