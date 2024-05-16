# this script creates robot control class that takes information from behavior script. 

import logging
import math
from pprint import pprint

import numpy as np
import omni
import omni.kit.pipapi
import omni.physx.scripts.utils as pxutils
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.scripting import BehaviorScript
from pxr import (Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, UsdUtils)

try:
    import zmq
except ModuleNotFoundError:
    omni.kit.pipapi.install("zmq")
    import zmq

logger = logging.getLogger(__name__)
commands = []


class RobotControl(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")

        self.robot = None

        self.context = zmq.Context()

        self.sock = self.context.socket(zmq.PUB)
        self.sock.connect('tcp://130.202.141.68:5560')

        #socket to pi in rpl tcp://146.137.240.73:5560
        # defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        # omni.usd.get_context().get_stage()

        self.had_first_update = False

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

        self.context.destroy()

    def on_play(self):
        logger.info(f"{__class__.__name__}.on_play()->{self.prim_path}")

        #logger.warn("UR Playing")

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
        if not self.had_first_update:
            self.on_first_update(current_time, delta_time)
        #logger.warn("Updating")
        joints = self.robot.get_joint_positions()
        joints = [str(j).encode() for j in joints]
        # joints = [*joints[:5], b'-4.7']
        #logger.warn(f'ov joints: {joints}')

        self.sock.send_multipart([b'Set', *joints])
        #logger.warn("Message Sent")
        
        
        #commands.append(*joints)
        #for command in commands:
        #    self.sock.send_multipart([b'1234magic5678', b'Set', command])

        # self.robot.apply_action(
        #     ArticulationAction(joint_positions=np.array(joints))
        # )
