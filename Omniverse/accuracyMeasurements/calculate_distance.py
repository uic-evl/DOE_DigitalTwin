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


logger = logging.getLogger(__name__)
commands = []

def distance(x1, y1, z1, x2, y2, z2):
    d = 0.0
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return d

# This should be attached to the CONTROL arm

class TargetControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        dc = _dynamic_control.acquire_dynamic_control_interface()

        # Get OVIS virtual endpoint
        # TO DO: don't hard code this path
        self.v_endpoint = self.stage.GetPrimAtPath("/World/firefighter/joint6_flange/pointTest").GetAttribute("endpoint")

        # Get Unity virtual endpoint
        # TO DO: don't hard code this path
        self.u_endpoint = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("unity_endpoint")

        # Get real endpoint 
        # TO DO: don't hard code this path
        self.r_endpoint = self.stage.GetPrimAtPath("/World/Real_Report/joint6_flange/pointTest").GetAttribute("endpoint")

        self.public_distance = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("public_distance")


    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")


    def on_play(self):
        pass

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False


    def on_update(self, current_time: float, delta_time: float):
        self.v = self.v_endpoint.Get()
        self.r = self.r_endpoint.Get()
        self.u = self.u_endpoint.Get()
        
        #dist = distance(self.v_endpoint[0], self.v_endpoint[1], self.v_endpoint[2], self.r_endpoint[0], self.r_endpoint[1], self.r_endpoint[2])
        dist1 = distance(self.v[0], self.v[1], self.v[2], self.r[0], self.r[1], self.r[2])

        dist2 = distance(self.v[0], self.v[1], self.v[2], self.u[0], self.u[1], self.u[2])

        self.public_distance.Set(dist1)

        # Converted to cm
        #logger.warn(str(dist * 100))

        print1 = "OVIS-Real Distance: " + str(dist1 * 100)
        #print2 = "OVIS-Unity Distance: " + str(dist2 * 100)
        #print_final = print1 + " | " + print2

        #logger.warn(print1)
        

        

        
