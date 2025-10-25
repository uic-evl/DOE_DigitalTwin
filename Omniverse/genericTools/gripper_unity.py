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

import zmq

from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)

logger = logging.getLogger(__name__)

'''
This script manages the state of the gripper if the UI is UNITY

In this case, the grip state is recieved as a command from Unity. 
Uses ZMQ to talk to Unity. 
'''
class GripConsumer(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.update_subscription = False
        self.prev_gripper_pos = 0

        # Reference to "switch_state" Attribute. 
        self.grip_state = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("gripper_state")

        # ZMQ Networking for Gripper Subscriber
        self.grip_ip = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("grip_ip").Get() # same as target ip
        self.grip_port = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("grip_port").Get() 

        # ZMQ Networking
        self.context = zmq.Context()
        self.sock = None  # Initialize the socket variable

    def on_pause(self):
        logger.warn("SwitchController Pause")

    def on_play(self):
        logger.warn("SwitchController Play")

        # Open ZMQ Socket
        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect(f'tcp://{self.grip_ip}:{self.grip_port}')
        self.sock.subscribe('')
        logger.warn("Gripper socket Set")

        # Default gripper state
        self.grip_state.Set(0)

    def on_stop(self):
        logger.warn("SwitchController Stop")

        if self.sock:
            self.sock.close()

    def on_update(self, current_time: float, delta_time: float):
        try:
            message = self.sock.recv_string(flags=zmq.NOBLOCK)
            
            #logger.warn(message)

            # String format from Unity Producer is subject to change based on the implmentation 
            # of the IDataProducer GameObject given to the Unity Producer. 

            x = float(message)

            # when message is 0, open
            if x == 0:
                self.grip_state.Set(0)
            # when message is 1, closed
            elif x > 0:
                self.grip_state.Set(1) 

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass  # no message was ready (yet!)
            else:
                logger.error(f"ZMQ Error: {str(e)}")

    def on_destroy(self):
        logger.warn(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        if self.sock:
            self.sock.close()
        self.context.destroy()