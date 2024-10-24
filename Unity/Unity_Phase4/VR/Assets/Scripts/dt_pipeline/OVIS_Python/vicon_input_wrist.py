import logging
import time
import omni.usd
from pxr import Usd, Gf, UsdGeom, Sdf
import omni.physx.scripts.utils as pxutils
from omni.physx.scripts import physicsUtils
from omni.isaac.dynamic_control import _dynamic_control
from omni.kit.scripting import BehaviorScript
from pxr import Gf, UsdGeom
import sys
import os
import math

# Vicon SDK
# If you have not already added the vicon_dssdk folder to your system path run the line below.
# Once you have run this, you will not need to run it again.
# Double check this path 
#sys.path.insert(0, os.path.abspath('/AppData/Local/Packages/PythonSoftwareFoundation.Python.3.11_qbz5n2kfra8p0/LocalCache/local-packages/Python311/site-packages'))
from vicon_dssdk import ViconDataStream

logger = logging.getLogger(__name__)
commands = []

class WristTargetControl(BehaviorScript):
    VICON_TRACKER_IP = ""
    WRIST_OBJECT_NAME = "ER300_Wrist"

    def on_init(self):
        logger.info(f"{__class__.__name__}.on_init()->{self.prim_path}")
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        # Initialize Vicon client
        self.client = ViconDataStream.Client()

        # Connect to Vicon tracker
        try:
            self.client.Connect(self.VICON_TRACKER_IP)
            logger.info('Connected to Vicon Tracker')
            logger.info('Version' + str(self.client.GetVersion()))

            # Check setting the buffer size works
            self.client.SetBufferSize(1)

            # Enable all the data types
            self.client.EnableSegmentData()
            self.client.EnableMarkerData()
            self.client.EnableUnlabeledMarkerData()
            self.client.EnableMarkerRayData()
            self.client.EnableDeviceData()
            self.client.EnableCentroidData()

        except ViconDataStream.DataStreamException as e:
            logger.error('Failed to connect to Vicon tracker: ' + str(e))
            self.client = None

    def on_destroy(self):
        logger.info(f"{__class__.__name__}.on_destroy()->{self.prim_path}")
        if self.client:
            self.client.Disconnect()

    def on_play(self):
        logger.warn("Vicon tracker initialized")

    def on_stop(self):
        follow_target = omni.usd.get_context().get_stage().GetPrimAtPath(str(self.prim_path))
        logger.info(f"{__class__.__name__}.on_stop()->{self.prim_path}")
        translation = Gf.Vec3d(0, 0, 0)
        rotation = Gf.Vec3f(0, 0, 0)  # Assuming XYZ Euler angles
        translate_op = follow_target.GetAttribute('xformOp:translate')
        translate_op.Set(Gf.Vec3f(translation))

        # Apply rotation as XYZ Euler angles
        rot_att = follow_target.GetAttribute("xformOp:rotateXYZ")
        rot_att.Set(Gf.Vec3f(rotation))

    def on_update(self, current_time: float, delta_time: float):
        if not self.client:
            logger.warn("Vicon client is not initialized")
            return

        try:
            HasFrame = False
            timeout = 50
            while not HasFrame:
                logger.info('.')
                try:
                    if self.client.GetFrame():
                        HasFrame = True
                    timeout = timeout - 1
                    if timeout < 0:
                        logger.info('Failed to get frame')
                        return
                except ViconDataStream.DataStreamException as e:
                    self.client.GetFrame()

            subjectNames = self.client.GetSubjectNames()
            for subjectName in subjectNames:
                logger.info(subjectName)
                segmentNames = self.client.GetSegmentNames(subjectName)
                for segmentName in segmentNames:
                    segmentChildren = self.client.GetSegmentChildren(subjectName, segmentName)
                    for child in segmentChildren:
                        try:
                            logger.info(f"{child} has parent {str(self.client.GetSegmentParentName(subjectName, segmentName))}")
                        except ViconDataStream.DataStreamException as e:
                            logger.warn('Error getting parent segment', e)
                    
                    # Get translation and rotation from Vicon
                    v_translation, v_translation_quality = self.client.GetSegmentGlobalTranslation(subjectName, segmentName)
                    v_rotation, v_rotation_quality = self.client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)
                    logger.info("translation stuff: "+ str(v_translation))
                    logger.info("rotation stuff: " + str(v_rotation))

                    # Track the wrist object (ViconCubeWrist)
                    if subjectName == self.WRIST_OBJECT_NAME:
                        # Convert Vicon coordinates to Omniverse coordinates
                        # Adjust the translation and rotation for vertical orientation
                        translation = Gf.Vec3d(v_translation[0] * 0.001, v_translation[1] * 0.001, v_translation[2] * 0.001) - Gf.Vec3d(0.006, 0.005, 0.06)
                        rotation = Gf.Vec3f(math.degrees(v_rotation[0]), math.degrees(v_rotation[1]), math.degrees(v_rotation[2]))  # Convert radians to degrees

                        # Apply transformation to the ViconCubeWrist
                        follow_target = omni.usd.get_context().get_stage().GetPrimAtPath(str(self.prim_path))
                        if follow_target:
                            # Apply translation
                            translate_op = follow_target.GetAttribute('xformOp:translate')
                            if not translate_op:
                                translate_op = follow_target.CreateAttribute('xformOp:translate', Sdf.ValueTypeNames.Float3, False)
                            translate_op.Set(Gf.Vec3f(translation))

                            # Apply rotation as XYZ Euler angles
                            rot_att = follow_target.GetAttribute("xformOp:rotateXYZ")
                            if not rot_att:
                                rot_att = follow_target.CreateAttribute("xformOp:rotateXYZ", Sdf.ValueTypeNames.Float3, False)
                            rot_att.Set(rotation)

        except ViconDataStream.DataStreamException as e:
            logger.warn('Handled data stream error' + str(e))
