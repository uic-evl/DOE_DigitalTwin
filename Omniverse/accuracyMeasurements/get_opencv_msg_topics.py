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
from omni.isaac.core.prims import XFormPrim
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.scripting import BehaviorScript
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)
import zmq

# This script should be placed on the IK Target in the OVIS Scene.
# It gets the IK Target position and rotation from Unity
# Applies the proper transformation, then updates the IK Target
# in Omniverse.

# Make sure to add and populate the "target_ip" and "target_port" attributes

logger = logging.getLogger(__name__)
commands = []

class GetTagOrigin(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        dc = _dynamic_control.acquire_dynamic_control_interface()
        
        # Testing initial pos extraction
        '''
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        trans = pose.ExtractTranslation()
        #logger.warn(trans)
        '''

        self.curr_prim = XFormPrim(prim_path=str(self.prim_path))
        self.endpoint_prim = XFormPrim(prim_path=str("/World/cv_endpoint"))

        # ZMQ Networking for Target Subscriber
        self.target_ip = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("target_ip").Get()
        self.target_port = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("target_port").Get()

        self.topic = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("topic").Get()
        self.isOrigin = self.stage.GetPrimAtPath(self.prim_path).GetAttribute("isOrigin").Get()

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
        self.sock.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        logger.warn(f"ZMQ socket for {self.prim_path} Set on topic {self.topic}")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False
        if self.sock:
            self.sock.close()

    def on_update(self, current_time: float, delta_time: float):        
        try:
            message = self.sock.recv_string(flags=zmq.NOBLOCK)
            
            topic, payload = message.split(" ", 1)
            logger.warn(f"{topic} -> {payload}")
            
            # Parse message string "payload"
            # Format: x,y,z,rw,rx,ry,rz

            # Camera pose from origin tag
            xform_data = payload.split(",")

            #logger.warn(xform_data)

            offset = 0.133
            #if(self.isOrigin):
            #    offset = 0.13

            x = (float(xform_data[0])) 
            y = (float(xform_data[1])) - offset
            z = (float(xform_data[2])) 

            qw = float(xform_data[3])
            qx = float(xform_data[4]) 
            qy = float(xform_data[5]) 
            qz = float(xform_data[6])
            

            self.curr_prim.set_world_pose(
                position=np.array((x,y,z)),
                orientation=np.array((qx,qy,qz,qw))
            )

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass  # no message was ready (yet!)
            else:
                logger.error(f"ZMQ Error: {str(e)}")