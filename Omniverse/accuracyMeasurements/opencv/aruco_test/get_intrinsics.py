import depthai as dai

# Get camera intrinsics
# Connect to device
with dai.Device() as device:
    # Read calibration data from EEPROM
    calibData = device.readCalibration()
    
    # Get intrinsics for the Left camera at a specific resolution
    intrinsics = calibData.getCameraIntrinsics(
        dai.CameraBoardSocket.LEFT, 
        resizeWidth=640, 
        resizeHeight=400
    )
    
    print("Intrinsic Matrix:")
    print(intrinsics)