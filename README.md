# EVL Digital Twin Efforts

## Overview

For this project, our goal is to create a "digital twin" of a given robot so that it can be controlled remotely through a virtual reality (VR) application. At its simplest, this will involve communicating between ROS2 and some VR-capable application.

Software:
* NVIDIA Omniverse (OV)
* Unreal Engine
* Unity
* ROS2 (Humble)

Hardware:
* HMD
    * Oculus Quest 2
* 2 [iRobot Create3](https://iroboteducation.github.io/create3_docs/) Roombas
* Intel NUC with Windows (Named Discovery)
* Intel NUC with Ubuntu

## Getting Started

### ROS2

As this project hinges on ROS2, first step will be to install and familiarize yourself with ROS2 Humble. 

If you are using Windows 10 or Linux, you can install ROS2 directly to you machine. Follow [this guide](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html) for Windows 10, and [this guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for Linux.

If you are using Windows 11, you will need to run ROS2 within WSL. **If you are using WSL/Win11, you will not be able to connect to external hardware, such as the Create3s, over a network.** Simulation and connection to Unity, however, will still work fine. 

Steps to install on Ubunutu (including WSL):
1. If you need it, install Ubuntu 22 through WSL. You should be able to do this on the Windows store.
2. [Install ROS2 Humble following this guide.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
    * make sure you get the desktop installation AND the dev tools:
            `sudo apt install ros-humble-desktop`
            `sudo apt install ros-dev-tools`
3. To make using ROS2 a bit easier, you can run `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` so that the ROS2 environment is sourced automatically when your shell starts. 
4. (Optional but recommended) Install [Xming X server](http://www.straightrunning.com/XmingNotes/) so that you can launch GUI applications from WSL. ROS2 has several handy GUI features, and the Create3 Robot makes use of these for some simulation tools. 

Once you have ROS2, I recommend following their turtlesim tutorial, as it gives a good overview of ROS2 terms that will be relevant to this project. TLDR: ROS2 is a system where a bunch of distributed scripts called **nodes** share data by publishing/subscribing to **topics**. These scripts can be written in C++ or Python.

### iRobot Create3

The iRobot Create3 is an educational robot that can be controlled using either their Python API or ROS2. We are going to be using ROS2. You can connect to the robot with the following methods:
* Bluetooth:
    * iRobot coding app 
    * https://python.irobot.com/ (Chrome only)
* Create3 wifi access point
    * Hold down the two buttons to either side of the power button until the ring light turns blue. 
    * Next, on a computer capable of wifi connections, look for a network that named something like "Create-NNNN"
    * Connect to th network, then navigate to `http://192.168.10.1`to access the robot's webserver
* Shared wifi network
    * From the directions above, you can use the robot's webserver page to connect it to a different wifi network
    * If the robot and computer are on the same network, you can find the robot's ip through your method of choice (`ipconfig`, or `arp -a`) and navigate to it. 
    * On the EVL-IoT network, the Create3's IP is `10.0.0.21`.
    * On the GL.inet mini router, the Create3ss IPs are `192.168.8.240` or `192.168.8.124`.

iRobot provides some [sample nodes](https://github.com/iRobotEducation/create3_examples/tree/humble) for controlling the robot. Once you have confirmed that your ROS2 `ros2 topic list`  command shows the topics from the Create3, you should be able to run these nodes to control the robot. 
 
### Network Notes

Currently, the only reliable way to connect ROS2 and the iRobot Create3 is using the wifi connection to the GL.inet mini-router. If the computer and the robot are both on the GL.inet network, and `ros2 topic list` does not print any topics containing the `/robot_1` or `/robot_2` prefix, you max need to restart the robot's ROS2 application, which can be done through the page at the IP addresses above. 

## Next Steps

Generally speaking, our next steps are as follows:
* Investigate and compare VR platforms (OV, Unreal, Unity) and test their connection with ROS2
    * OV: Direct connection with ROS2 through Isaac Sim
    * Unity: Understand how to control/represent the robot in Unity using the Unity Robotics Hub tools with ROS2.   
    * Unreal: Open Question

* Formalize experiments.
