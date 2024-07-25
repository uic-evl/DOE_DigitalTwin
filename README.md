# EVL Digital Twin Efforts

For more detailed documentation and specific user guides, please visit our [wiki!](https://github.com/uic-evl/digital-twin/wiki)

## Overview

For this project, our goal is to create a "digital twin" of a robot (or robots). This application must support a **bidirectional** flow of data, where the virtual simulation/application may affect the physical hardware state, and the physical hardware state may be reflected in the virtual simulation/application. 

As we develop this application, we will test across hardware, development, and software platforms to compare current methods for creating such digital twins. 

Software:
* NVIDIA Omniverse (OV)
  * Isaac Sim (OVIS)
* Unreal Engine 5
* Unity
* ROS2
  * Create3: Humble
  * ER Arms: Galactic 

Hardware:
* HMD
    * Oculus Quest 2
    * Apple Vision Pro
* 2 [iRobot Create3](https://iroboteducation.github.io/create3_docs/) Roombas
* [Elephant Robotics MyCobot 280 JN](https://www.elephantrobotics.com/en/mycobot-en/)
* [Elephant Robotics MyArm 300 Pi](https://shop.elephantrobotics.com/products/myarm)
* Windows 11 Machines:
   * Columbia (EVL)
   * Perserverance (ANL)
   * Curiosity (ANL)
   * Opportunity (ANL)
   * Spirit (ANL)
   * Sojourner (ANL)
* Ubuntu Machines:
   * Discovery (EVL) 
   * Ingenuity (ANL) 
* Raspberry Pi 2, 4, and 5
* Sensors
  * Vicon Cameras
  * Luxonis Camera 

## Current Work

To view our current progress, you can either reference the [open Issues](https://github.com/uic-evl/digital-twin/issues) or our [Experiments Tracker](https://github.com/uic-evl/digital-twin/wiki/Experiments).

Current Tasks:
* Unity
  * Add RabbitMQ Networking
  * Build NetMQ for Linux
  * Improve VR Scene
* Isaac Sim
  * Twin Create3
  * Add camera stream
  * ROS Bridge 
* Unreal Engine 5
  * Develop VR User Interface

## Unity Sample Project 

There are three sample Unity projects. More detail about using these projects can be found [in the /Unity folder](https://github.com/uic-evl/digital-twin/blob/main/Unity/README.md), as well as in each project's README. 

The [Unity Robotics Hub Repository](https://github.com/Unity-Technologies/Unity-Robotics-Hub) provides packages for Unity and ROS that allow users to setup a TCP connection between the two platforms. We have tested this and confirmed that their packages work to communicate between a Unity app and ROS2 on Windows 10, Ubuntu, and Win11/WSL. Instructions on how to set up a Unity project to exchange data with ROS can be found [on our wiki.](https://github.com/uic-evl/digital-twin/wiki/Connecting-Unity-to-ROS2-on-Ubuntu)

## Isaac Sim and Unreal Engine 5 Sample Project 

Isaac Sim is a robotics simulator within the NVIDIA Omniverse toolkit. We leverage the Omniverse-Unreal Connector to simulate the robot within Unreal Engine 5 in a Live Session. You can refer to [our guide](https://github.com/uic-evl/digital-twin/wiki/Simple-Digital-Twin-in-Omniverse-and-Unreal-Engine-5) on how to set this up with the Elephant Robotics arms (or, in theory, any arm with a URDF file). Necessary scripts can be found in [this repository.](https://github.com/uic-evl/digital-twin/tree/main/Omniverse_ER/demo_files)
