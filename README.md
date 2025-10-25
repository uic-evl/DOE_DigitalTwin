# EVL Digital Twin Efforts

For more detailed documentation and specific user guides, please visit our [wiki!](https://github.com/uic-evl/digital-twin/wiki)

## Overview

For this project, our goal is to create a "digital twin" of a robot (or robots). This application must support a **bidirectional** flow of data, where the virtual simulation/application may affect the physical hardware state, and the physical hardware state may be reflected in the virtual simulation/application. 

As we develop this application, we will test across hardware, development, and software platforms to compare current methods for creating such digital twins. 

Software:
* NVIDIA Omniverse (OV)
  * Isaac Sim (OVIS)
* Unity
* ROS2

Hardware:
* HMD
    * Oculus Quest 2
    * Apple Vision Pro
* 2 [iRobot Create3](https://iroboteducation.github.io/create3_docs/) Roombas
* [Elephant Robotics MyCobot 280 JN](https://www.elephantrobotics.com/en/mycobot-en/)
* [Elephant Robotics MyArm 300 Pi](https://shop.elephantrobotics.com/products/myarm)
* Sensors
  * IR Cameras
  * Luxonis 3D Camera
  * Intel RealSense D455

## Current Work

![DT Architecture Diagram](https://github.com/uic-evl/DOE_DigitalTwin/blob/restructure/wiki_images/diagram.png)

 Our current approach to creating a digital twin is to consider the simulation environment, the user interface, and the hardware as three distinct entities that can communicate with each other. We support three primary configurations of simulation and user interface platforms: Omniverse-only, Unity-only, and Omniverse-Unity. 

## Ongoing Tasks

To view our progress, you can reference the [open Issues](https://github.com/uic-evl/digital-twin/issues).

Tasks:
* Issac Lab (Gymnasium)
  * See [HotCellGym branch](https://github.com/uic-evl/DOE_DigitalTwin/tree/Isaac-Lab-Hotcell-Environment)
* Unity
  * Create unified Unity UI of all interaction types
* Scene Generation
  * Integrate multi-RealSense scene sensing     
