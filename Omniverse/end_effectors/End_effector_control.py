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

# MOTION_GEN_ALGO = 'RMPFlow'
logger = logging.getLogger(__name__)

PATH_BASE = Path("C:/Omniverse_Files/robot_data/MyArm300pi_EE_test3.usd") #absolute usd path

EE_name = '/gripper_base/gripper_controller'
switch_attr = "JWM:Switch_state"

class EndEffectorControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

    def on_play(self):
        logger.warn("EE Control Play")
        self.set_gripper_action(-19, 'pos') 

    def on_update(self, current_time: float, delta_time: float):
        self.Switch_state_action()

    def on_pause(self):
        logger.warn("EE Control Pause")

    def on_stop(self):
        #Reseting the gripper

        EE = self.stage.GetPrimAtPath(str(self.prim_path) + EE_name)
        EE_target_pos_prev = get_attribute_value(EE, "drive:angular:physics:targetPosition")
        omni.kit.commands.execute('ChangeProperty',
            prop_path=Sdf.Path('/World/firefighter/gripper_base/gripper_controller.drive:angular:physics:targetPosition'),
            value=0,
            prev=EE_target_pos_prev,
            target_layer=Sdf.Find(str(PATH_BASE)),
        )   

    def on_destroy(self):
        logger.warn(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def Switch_state_action(self):
        switch = self.stage.GetPrimAtPath(str(self.prim_path)+ '/Switch')
        switch_state = get_attribute_value(switch,switch_attr)

        if switch_state == 0:
            self.set_gripper_action(1,'pos')
            #logger.error('gripper will do nothing')

        elif switch_state == 1:
            #logger.warn(f"switch state is 1, the gripper is opening.")
            self.set_gripper_action( 1, 'pos')
        elif switch_state == 2:
            #logger.warn(f"switch state is 2, the gripper is closing.")
            self.set_gripper_action( -19, 'pos')
        elif switch_state == 3:
            #logger.warn(f"switch state is 3, the gripper is releasing.")
            #logger.error(f'Currently release functions just like closing')
            self.set_gripper_action( -19, 'vel')
            return
        else:
            #logger.error(f'swtich state not found')
            pass


    def set_gripper_action(self, target_pos_new: int, type: str):

        EE = self.stage.GetPrimAtPath(str(self.prim_path) + EE_name)
        EE_target_pos_prev = get_attribute_value(EE, "drive:angular:physics:targetPosition")

        if type == 'pos':

            test = omni.kit.commands.execute('ChangeProperty',
                prop_path=Sdf.Path('/World/firefighter/gripper_base/gripper_controller.drive:angular:physics:targetPosition'),
                value=target_pos_new,
                prev=EE_target_pos_prev,
                target_layer=Sdf.Find(str(PATH_BASE)),
            )
            if not test:
                pass
                #logger.error(f"The target position was not changed")

        elif type == 'vel':
            old_vel = get_attribute_value(EE, "drive:angular:physics:targetPosition")
            new_vel = target_pos_new * 0.8
            test1 = omni.kit.commands.execute('ChangeProperty',
                prop_path=Sdf.Path('/World/firefighter/gripper_base/gripper_controller.drive:angular:physics:targetPosition'),
                value=EE_target_pos_prev + 0.2,
                prev=EE_target_pos_prev,
                target_layer=Sdf.Find(str(PATH_BASE)),
            )

            test2 = omni.kit.commands.execute('ChangeProperty',
                prop_path=Sdf.Path('/World/firefighter/gripper_base/gripper_controller.drive:angular:physics:targetVelocity'),
                value=new_vel,
                prev=old_vel,
                target_layer=Sdf.Find(str(PATH_BASE)),
            )
            if not test1 and test2:
                pass
                #logger.error(f"The target VELOCITY was not changed")



def get_attribute_value(prim: Usd.Prim, attribute_name: str):
    attr = prim.GetAttribute(attribute_name)
    if not attr.IsValid():
        raise AttributeError(f"Attribute {attribute_name} not found on prim {prim.GetPath()}")
    return attr.Get()