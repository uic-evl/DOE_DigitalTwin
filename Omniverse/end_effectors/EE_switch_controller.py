import omni.usd
import omni
import omni.kit.pipapi
import omni.kit.commands
import omni.physx.scripts.utils as pxutils
from omni.physx.scripts import physicsUtils
from omni.kit.scripting import BehaviorScript
from omni.physx import get_physx_scene_query_interface

import numpy as np
import logging
import math
from pprint import pprint
import time
import typing
from pathlib import Path

import os
import asyncio

from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)

logger = logging.getLogger(__name__)

'''
Reminder: There are things to update for each file. You need to update script path, for each use locate the colors that are used on the switch (Omnipbr) to find the 

Purpose: To create an example end effector control method for VR Isotope DT project or like projects.  
Author: James Morrissette
Date: 7/8/24
Email:jameswm@iastate.edu

'''
#any paths that are worth editing are here. Script should generate and bind materials as needed
PATH_BASE = Path("C:/Users/halle/Documents/DigitalTwin/gripper_stage.usd") #absolute usd path
switch_path =("/World/firefighter/Switch")
mesh_path =("/World/firefighter/Switch")
material_path = "/World/Looks/OmniPBR"#path to material for color change 
state_name = "JWM:Switch_state"#state of the switch

class SwitchControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.update_subscription = False
        self.prev_state = 0

    def on_pause(self):
        logger.warn("SwitchController Pause")

    def on_play(self):
        logger.warn("SwitchController Play")
        self.option_select()

    def on_stop(self):
        logger.warn("SwitchController Stop")

        logger.warn(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        
        # update each line makred with a # using the command window
        ColorChange = omni.kit.commands.execute('ChangeProperty',
                        prop_path=Sdf.Path('/World/Looks/OmniPBR/Shader.inputs:diffuse_color_constant'),
                        value=Gf.Vec3f(0.2,0.2,0.2),
                        prev=Gf.Vec3f(0.2,0.2,0.2),
                        target_layer=Sdf.Find(str(PATH_BASE)),
                        ) 
        switch_attr: Usd.Attribute = self.curr_prim.GetAttribute(state_name)
        switch_attr.Set(0)

        logger.warn(f"The gripper has been set to defult and the color is grey {ColorChange}")

    def on_update(self, current_time: float, delta_time: float):
        self.option_select()

    def option_select(self):    
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        x,y,z = pose.ExtractTranslation()
        switch_attr: Usd.Attribute = self.curr_prim.GetAttribute(state_name)
        send_attr: Usd.Attribute = self.curr_prim.GetAttribute("toSend")

        if  y > .2 and x == 0: # option 3, release
            #logger.warn("The gripper will release")
            switch_attr.Set(3)
            assign_material_color(self,'green',__class__.__name__ )

        elif x == 0: # option 0, do nothing
            #logger.warn("The gripper will do nothing")
            switch_attr.Set(2)
            assign_material_color(self,'grey',__class__.__name__ )

        elif x > 0:# option 1, open
            #logger.warn("The gripper will Open")
            switch_attr.Set(0)
            assign_material_color(self,'blue',__class__.__name__ )

        elif x < 0:# option 2, close
            #logger.warn("The gripper will Close")
            switch_attr.Set(1)
            assign_material_color(self,'red',__class__.__name__ )

        # New state, so we send
        if self.prev_state != switch_attr.Get():
            send_attr.Set(True)

        self.prev_state = switch_attr.Get()


    def switch_attr_check(self):
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)

        switch_attr: Usd.Attribute = self.curr_prim.GetAttribute(state_name)
        if not switch_attr.IsValid():
            # do not need to update
            omni.kit.commands.execute(
                "CreateUsdAttributeCommand",
                prim=self.curr_prim,
                attr_name=state_name,
                attr_type=Sdf.ValueTypeNames.Int,
            )

            #logger.warn("Switch attribute has been created and set to:" + str(0))
            #logger.error(str(switch_attr))

        else:
            prev_value = get_attribute_value(self.curr_prim, state_name)
            #logger.warn("Switch attribute"+str(switch_attr)+"has already been created")
            #logger.warn("The previous Switch value is:" + str(prev_value))

            switch_attr.Set(0)
            new_value = get_attribute_value(self.curr_prim, state_name)
            #logger.warn("Switch is set to: "+str(new_value))
                
        return switch_attr
        
    def on_destroy(self):
        logger.warn(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

def get_attribute_value(prim: Usd.Prim, attribute_name: str):
    """
    See: https://openusd.org/release/api/class_usd_attribute.html
    Args:
        prim: The prim owner of the attribute.
        attribute_name: The name of the attribute to retrieve.
    Return:
        The value of the attribute, see https://openusd.org/release/api/_usd__page__datatypes.html
        for the return types.
        For example, for `float3`, the return type will be `Gf.Vec3f`.
    """
    attr = prim.GetAttribute(attribute_name)
    if not attr.IsValid():
        raise AttributeError(f"Attribute {attribute_name} not found on prim {prim.GetPath()}")

    return attr.Get()



def assign_material_color(self,color: str,DontChange):
    '''
    adapted from http://omniverse-docs.s3-website-us-east-1.amazonaws.com/omni.kit.usd_docs/1.1.1/USD%20in%20Kit.html
    '''

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(mesh_path)

    if not prim:
        logger.error(f"Prim not found at {mesh_path}")
        return
    
    if not stage.GetPrimAtPath(material_path):
        omni.kit.commands.execute('CreateAndBindMdlMaterialFromLibrary',
            mdl_name='OmniPBR.mdl',
            mtl_name='OmniPBR',
            mtl_created_list=None,
            bind_selected_prims=True)
        logger.error("Creating a new omnipbr")
        

    bound_material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if not bound_material:
        logger.error(f"Material not bound to the prim at {mesh_path}")
        return
    
    logger.info(f"Bound Material is: {bound_material}")

    chosen_color = pick_color(color)
    # seting the color 
    ColorChange = omni.kit.commands.execute('ChangeProperty',
                        prop_path=Sdf.Path('/World/Looks/OmniPBR/Shader.inputs:diffuse_color_constant'),
                        value=chosen_color,
                        prev=chosen_color,
                        target_layer=Sdf.Find(str(PATH_BASE)),
                        )#

    if not ColorChange:
        logger.error("unable to change diffuse_color_constant ")



#asyncio.ensure_future(assign_red_material()) not needed yet but to remind me of format

def grey(): #this is for option 0
    print("Action for state 1")
    return Gf.Vec3f(0.5, 0.5, 0.5)


def blue(): #this is for option 1
    print("Action for state 2")
    return Gf.Vec3f(0.0, 0.0, 1.0)


def red(): #this is for option 2
    print("Default action")
    return Gf.Vec3f(1.0, 0.0, 0.0)


def green(): #this is for option 3
    print("Default action")
    return Gf.Vec3f(0.0, 1.0, 0.0)

# Define a dictionary mapping states to actions
colors = {
    'grey': grey,
    'blue': blue,
    'red': red,
    'green': green,
}

# Function to handle the state
def pick_color(color):
    # Get the action from the dictionary, default to default_action if the state is not found
    action = colors.get(color, grey)
    chosen_color = action()
    #logger.warn(f"The color chose is: {action.__name__} which has value: {chosen_color}")
    return chosen_color

#pick_color('grey') # defult use example