import depthai as dai

deviceInfos = dai.Device.getAllAvailableDevices()
print("=== Found devices: ", deviceInfos)
