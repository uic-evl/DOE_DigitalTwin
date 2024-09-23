import threading
import logging
import math
from pprint import pprint
import time
import typing
import ast
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

import pika
logger = logging.getLogger(__name__)
position_data=[]
rotation_data=[]

class TargetControl(BehaviorScript):

    def on_init(self):

        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        dc = _dynamic_control.acquire_dynamic_control_interface()
        # Testing initial pos extraction
        stage = omni.usd.get_context().get_stage()
        self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        trans = pose.ExtractTranslation()
        logger.warn(trans)
        
        self.credentials = pika.PlainCredentials('hema', 'hema')
        self.parameters = pika.ConnectionParameters('10.0.0.119', 5672, '/', self.credentials)
        


    def on_play(self):
        self.connection = pika.BlockingConnection(self.parameters)

        self.channel = self.connection.channel()

        self.channel.queue_declare(queue='logs')
        # def callback(ch, method, properties, body):
        #     logger.error(f" [x] Received {body}")
        # self.channel.basic_consume(queue='logs', on_message_callback=callback, auto_ack=True)

        logger.error(' [*] Waiting for messages. To exit press CTRL+C')
        logger.error(self.channel)
        

        def callback(ch, method, properties, body):
                global position_data
                global rotation_data
                logger.error(f" [x] Received {body}")
                message = body.decode('utf-8')
                data= ast.literal_eval(message)
                logger.error(f" [x] Received {data}")
                position_data = list(data[0])
                rotation_data = list(data[1])
                logger.error(f" [x] Received {len(position_data)}")
                self.channel.stop_consuming()
                # setPositions(self)
        self.channel.basic_consume(queue='logs', on_message_callback=callback, auto_ack=True)
        self.channel.start_consuming()
        # self.sock = self.context.socket(zmq.SUB)
        # self.sock.connect('tcp://192.168.1.195:12344')
        # self.sock.subscribe('')
        # logger.error("Rabbit mq Set")
    # def print_square(slef):
             
    #         self.channel.start_consuming()

        # def setPositions(self):
        #     if(len(position_data)>0):
        #         logger.error(str(position_data)+" "+str(rotation_data))
        #         x = float(position_data[0]) * -100
        #         y = float(position_data[1]) * 100
        #         z = float(position_data[2]) * 100

        #             #logger.warn(position_data)
            
        #             #r = float(rotation_data[0])
        #             #i = float(rotation_data[1])
        #             #j = float(rotation_data[2])
        #             #k = float(rotation_data[3])

        #         rx = float(rotation_data[0]) 
        #         ry = float(rotation_data[1]) * -1.0
        #         rz = float(rotation_data[2]) * -1.0

        #             #physicsUtils.set_or_add_translate_op(self.curr_prim, (z,x,y))
        #             #physicsUtils.set_or_add_rotate_op(self.curr_prim, (rz,rx,ry))
        #             #physicsUtils.set_or_add_orient_op(self.curr_prim, Gf.Quatf(r,i,j,k))

        #             # Update transform using UsdGeom.XformCommonAPI
        #         xformable = UsdGeom.XformCommonAPI(self.curr_prim)

        #         rot_att = self.curr_prim.GetAttribute("xformOp:rotateXYZ")
        #         rot_att.Set(Gf.Vec3f(rz,rx,ry))

        #         xformable.SetTranslate((z, x, y))
        #         logger.error("Rabbit mq ")

    def on_update(self, current_time: float, delta_time: float):
        # logger.error("lol")
        


        def callback(ch, method, properties, body):
                global position_data
                global rotation_data
                # logger.error(f" [x] Received {body}")
                message = body.decode('utf-8')
                data= ast.literal_eval(message)
                # logger.error(f" [x] Received {data}")
                position_data = list(data[0])
                rotation_data = list(data[1])
                # logger.error(f" [x] Received {len(position_data)}")
                self.channel.stop_consuming()


        # logger.error(len(position_data))
        if(len(position_data)>0):
            # logger.error(str(position_data)+" "+str(rotation_data))
            x = float(position_data[0]) * -100
            y = float(position_data[1]) * 100
            z = float(position_data[2]) * 100

                #logger.warn(position_data)
        
                #r = float(rotation_data[0])
                #i = float(rotation_data[1])
                #j = float(rotation_data[2])
                #k = float(rotation_data[3])

            rx = float(rotation_data[0]) 
            ry = float(rotation_data[1]) * -1.0
            rz = float(rotation_data[2]) * -1.0

                #physicsUtils.set_or_add_translate_op(self.curr_prim, (z,x,y))
                #physicsUtils.set_or_add_rotate_op(self.curr_prim, (rz,rx,ry))
                #physicsUtils.set_or_add_orient_op(self.curr_prim, Gf.Quatf(r,i,j,k))

                # Update transform using UsdGeom.XformCommonAPI
            xformable = UsdGeom.XformCommonAPI(self.curr_prim)

            rot_att = self.curr_prim.GetAttribute("xformOp:rotateXYZ")
            rot_att.Set(Gf.Vec3f(rz,rx,ry))

            xformable.SetTranslate((z, x, y))
            # logger.error("Rabbit mq ")
            self.channel.basic_consume(queue='logs', on_message_callback=callback, auto_ack=True)
            self.channel.start_consuming()
        # try:
        #     t1 = threading.Thread(target=print_square, args=(self,))
       
        # except exception as e:
        #     logger.error("lol"+e)
    # def print_cube(num):
    #     credentials = pika.PlainCredentials('hema', 'hema')
    #     parameters = pika.ConnectionParameters('10.0.0.119', 5672, '/', credentials)
    # # connection = pika.BlockingConnection(pika.ConnectionParameters(host='127.0.0.1'))
    #     connection = pika.BlockingConnection(parameters)

    #     channel = connection.channel()

    #     channel.queue_declare(queue='logs')
    # # print(channel)

    #     def callback(ch, method, properties, body):
    #         logger.error(f" [x] Received {body}")

    #     channel.basic_consume(queue='logs', on_message_callback=callback, auto_ack=True)

    #     logger.error(' [*] Waiting for messages. To exit press CTRL+C')
    #     logger.error(channel)

    #     channel.start_consuming()
    #     logger.error("Cube: {}" .format(num * num * num))


    # def print_square(num):
    #     logger.error("Square: {}" .format(num * num))


    # t1 = threading.Thread(target=print_square, args=(10,))
    # t2 = threading.Thread(target=print_cube, args=(10,))

    # t1.start()
    # t2.start()

    # t1.join()
    # t2.join()

    # logger.error("Done!")   



