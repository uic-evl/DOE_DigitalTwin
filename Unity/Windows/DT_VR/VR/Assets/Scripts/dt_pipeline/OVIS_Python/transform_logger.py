import logging
import math
from pprint import pprint
import time
from datetime import datetime
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

PATH_BASE = Path("PATH/TO/PULLED/REPO")

class CustomLogger(BehaviorScript):
    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.frameNumber = 0

        # Open file stream
        self.filestream = open(str(PATH_BASE) + "/Unity/Unity_Phase4/VR/Assets/LogFiles/omni_out.txt", "w", encoding="utf-8")

        # Testing initial pos extraction
        #stage = omni.usd.get_context().get_stage()
        #self.curr_prim = stage.GetPrimAtPath(self.prim_path)
        #pose = omni.usd.get_world_transform_matrix(self.curr_prim)
        #trans = pose.ExtractTranslation()
        #logger.warn(trans)

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
       pass

    def on_stop(self):
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        self.had_first_update = False

        # Close file stream
        self.filestream.close()


    def on_update(self, current_time: float, delta_time: float):
        #logger.warn("hello from frame " + str(self.frameNumber))

        current_time = datetime.now()

        # Add time and frame number to data string 
        data = str(current_time) + "," + str(self.frameNumber)
        
        # Add transforms to data string
        link_base_path = "/World/firefighter/"

        for i in range(0,7):
            if(i == 0):
                link_name = "base"
            else:
                link_name = "joint" + str(i)
            
            stage = omni.usd.get_context().get_stage()
            my_link = stage.GetPrimAtPath(link_base_path + link_name)
            pose = omni.usd.get_world_transform_matrix(my_link)
            trans = pose.ExtractTranslation()

            reversed_ident_mtx = reversed(Gf.Matrix3d())
            rot = Gf.Vec3d(*reversed(pose.ExtractRotation().Decompose(*reversed_ident_mtx)))

            #link_data = str(trans[0]) + " " + str(trans[1]) + " " + str(trans[2]) + "," + str(rot[0]) + " " + str(rot[1]) + " " + str(rot[2])
            link_data = ",%.5f %.5f %.5f,%.5f %.5f %.5f" % (trans[0], trans[1], trans[2], rot[0], rot[1], rot[2])
            data = data + link_data

        data = data + "\n"
        self.filestream.writelines(data) 

        self.frameNumber = self.frameNumber + 1