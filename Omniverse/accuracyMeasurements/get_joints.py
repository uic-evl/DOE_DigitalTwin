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


logger = logging.getLogger(__name__)
commands = []

class TargetControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        dc = _dynamic_control.acquire_dynamic_control_interface()
        
        # ZMQ Networking for Joints Subscriber
        self.target_ip = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("target_ip").Get()
        self.target_port = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("target_port").Get()

        # TO DO: Add dynamic joint configuration 
        self.joint1 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint1_arm")
        self.joint2 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint2_arm")
        self.joint3 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint3_arm")
        self.joint4 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint4_arm")
        self.joint5 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint5_arm")
        self.joint6 = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("joint6_arm")

        self.context = zmq.Context()
        self.sock = None  # Initialize the socket variable

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")
        if self.sock:
            self.sock.close()
        self.context.destroy()

    def on_play(self):
        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect(f'tcp://{self.target_ip}:{self.target_port}')
        self.sock.subscribe('')
        logger.warn("ZMQ socket Set")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False
        if self.sock:
            self.sock.close()

    def on_update(self, current_time: float, delta_time: float):
        try:
            message = self.sock.recv_string(flags=zmq.NOBLOCK)
            
            #logger.warn(message)

            message = message.replace('(', '')
            message = message.replace(')', '')
            message = message.replace('[', '')
            message = message.replace(']', '')
            message = message.replace("'", '')

            joint_list = message.split(",")

            # TO DO: Add dynamic joint configuration 
            self.joint1.Set(float(joint_list[0]))
            self.joint2.Set(float(joint_list[1]))
            self.joint3.Set(float(joint_list[2]))
            self.joint4.Set(float(joint_list[3]))
            self.joint5.Set(float(joint_list[4]))
            self.joint6.Set(float(joint_list[5]))


        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass  # no message was ready (yet!)
            else:
                logger.error(f"ZMQ Error: {str(e)}")