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
* [iRobot Create3](https://iroboteducation.github.io/create3_docs/)
* Intel NUC (Named Discovery)

## Getting Started

### ROS2

As this project hinges on ROS2, first step will be to install and familiarize yourself with ROS2 Humble. 

ROS2 is supported on Windows 10, but I have never used it (feel free to investigate this, though), and the VR workstation (Discovery, the Intel NUC) is a Windows 11 machine. So, currently, the plan is to use WSL. 

**I am assuming you have a windows machine, as that's what we're gonna use for VR. If you have a Mac, you should use the Intel NUC instead.**

Steps:
1. If you don't have it already, install Ubuntu 22 through WSL. You should be able to do this on the Windows store.
2. [Install ROS2 Humble following this guide.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
    * make sure you get the desktop installation AND the dev tools:
            `sudo apt install ros-humble-desktop`
            `sudo apt install ros-dev-tools`
3. (Optional but recommended) Install [Xming X server](http://www.straightrunning.com/XmingNotes/) so that you can launch GUI applications from WSL. ROS2 has several handy GUI features, and the Create3 Robot makes use of these for some simulation tools. 

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
    * On the GL.inet mini router, the Create3's IP is `192.168.8.240`.

## Next Steps

Generally speaking, our next steps are as follows:
* Investigate and compare VR platforms (OV, Unreal, Unity) and test their connection with ROS2
    * OV: Direct connection with ROS2 through Isaac Sim
    * Unity: Open Question
    * Unreal: Open Question
* Troubleshoot Create3 ROS2 networking issue
    * At present, we can't get the ROS2 running on Discovery's WSL to see the Create3's ROS2 topics. 
    * We have a GL.inet mini router that we are using to place Discovery and the Create3 on their own wifi network.
        * You will need the admin password and wifi access key to use the mini-router. 
    
