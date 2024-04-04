### Windows 10 Unity-ROS Connection

To run this project, you must have a Unity Editor and ROS2 workspace configured according to the guide [on our wiki](https://github.com/uic-evl/digital-twin/wiki/Connecting-Unity-to-ROS2-on-Ubuntu-and-Windows-10).
The scripts in this package Subscribe to the "Imu" topic, which uses the ROS2 common message type `sensor_msgs/Imu`. We tested it using a Create3 roomba connected over wifi. 
