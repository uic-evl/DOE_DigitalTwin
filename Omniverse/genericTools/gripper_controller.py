import omni.usd
import omni
import omni.kit.commands
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
from omni.physx.scripts import physicsUtils
from omni.kit.scripting import BehaviorScript
from omni.physx import get_physx_scene_query_interface

import numpy as np
import logging
import math
from pprint import pprint
import time
from typing import Tuple
from pathlib import Path
import sys
import os

from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)
logger = logging.getLogger(__name__)

class EndEffectorControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.ee_prim = self.stage.GetPrimAtPath(str(self.prim_path)+"/gripper_base/gripper_controller")
       
        # Create or get open/closed position attributes
        self.open_position_attr = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("gripper_open_position")
        self.closed_position_attr = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("gripper_closed_position")
    
       
        # Get grip_state attribute (not switch_state)
        self.grip_state = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("gripper_state") #attribute on firefighter, gripstate 0 or 1
    
    def on_play(self):
        logger.warn("EE Control Play")
    
    def on_update(self, current_time: float, delta_time: float):
        # Get switch state
        grip_state_value = self.grip_state.Get()
        
        # Get open/close target values from attributes
        open_position = self.open_position_attr.Get()
        closed_position = self.closed_position_attr.Get()
        
        # Update joint
        # Open
        if(grip_state_value == 0):
            self.ee_prim.GetAttribute("drive:angular:physics:targetPosition").Set(open_position)
        # Closed
        elif(grip_state_value == 1):
            self.ee_prim.GetAttribute("drive:angular:physics:targetPosition").Set(closed_position)
    
    def on_pause(self):
        logger.warn("EE Control Pause")
    
    def on_stop(self):
        #Reseting the gripper
        pass

    def on_destroy(self):
        logger.warn(f"{__class__.__name__}.on_destroy()->{self.prim_path}")