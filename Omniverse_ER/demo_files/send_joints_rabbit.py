import logging
import math
from pprint import pprint
import time
from datetime import datetime


import numpy as np
import omni
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.scripting import BehaviorScript
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)
import os
import sys
import json


# try:
#     import zmq
# except ModuleNotFoundError:
#     omni.kit.pipapi.install("zmq")
#     import zmq


try:
    import pika
except ModuleNotFoundError:
    omni.kit.pipapi.install("pika")
    import pika

logger = logging.getLogger(__name__)
commands = []
json_angles=[]
temp="0,0,0,0"

class RobotControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

        self.robot = None

        # self.context = zmq.Context()

        # self.sock = self.context.socket(zmq.PUB)
        # #self.sock.connect('tcp://130.202.141.68:5560') #FOR ER LINK
        # self.sock.bind("tcp://*:12346") #FOR UNITY

        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='127.0.0.1'))
        self.channel = self.connection.channel()

        self.channel.queue_declare(queue='joints_str')
        # self.channel.queue_declare(queue='joints_str_unity')
        # channel.basic_publish(exchange='', routing_key='hello', body='Hello World!')
        # print(" [x] Sent 'Hello World!'")
        

        #socket to pi in rpl tcp://146.137.240.73:5560
        # defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        # omni.usd.get_context().get_stage()

        self.had_first_update = False
        self.filestream = open("D:/Hema/omni_out.txt", "w", encoding="utf-8")


    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        self.connection.close()

    def on_play(self):
        logger.info(f"{__class__.__name__}.on_play()->{self.prim_path}")


        # Python env debugging
        #mypath = os.path.abspath(zmq.__file__)
        #logger.warn(mypath)
        #logger.warn(sys.path)

        self.had_first_update = False

    def on_pause(self):
        logger.info(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")

        self.had_first_update = False

    def on_first_update(self, current_time: float, delta_time: float):
        self.had_first_update = True

        self.robot = Articulation(str(self.prim_path))
        self.robot.initialize()
  

    def on_update(self, current_time: float, delta_time: float):
        global temp
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        
        #joints = self.robot.get_joint_positions()
        #joints = [str(j).encode() for j in joints]
 
        #self.sock.send_multipart([b'Set', *joints])

        # Get joint positions from the robot
        current_time = datetime.now()
        data = str(current_time) + ","
        joints = self.robot.get_joint_positions()
        # temp=joints

        
        # Convert joint positions to strings and then to a single string
        joints_str = str([str(j) for j in joints])

        # if(joints_str!=temp):
        #             # logger.info(f"temp {type(temp)}")
        #             json_angles.append(joints_str)
        #             temp=str(joints_str)

        #         # json_angles.append(joints_str)
        #         # temp=joints_str
        json_angles.append(data+joints_str)
        json_object = json.dumps(json_angles, indent=4)
 
        # Writing to sample.json
        with open("D:\Hema\sample.json", "w") as outfile:
            outfile.write(json_object)

        # logger.info(f"joints_str {joints_str}")
        data = data + joints_str
        data = data + "\n"
        self.filestream.writelines(data) 
        
        
        #Send joint positions as a single string
        # try:
            # self.sock.send_string(joints_str)
        self.channel.basic_publish(exchange='', routing_key='joints_str', body=joints_str)

        # except zmq.ZMQError as exc:
        #     logger.warn(exc)