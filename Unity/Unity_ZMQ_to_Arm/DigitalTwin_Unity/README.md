# Arm ZMQ

## Overview 
This is a sample scene that uses the Unity Robotics Hub URDF importer to import the Elephant Robotics Arm and control it through Isaac Sim. 

This project requires an Isaac Sim counterpart scene. You can find this on Spirit's Nucleus under `/Projects/Digital Twin/unity_connection/pos_from_unity.usd`

ZMQ-Unity code adapted from [gench23's repo](https://github.com/gench23/unity-zeromq-client)

## Quickstart

1. Add the Unity project and open in the required editor. 
2. Open the "Bidirectional" scene from the "Assets/Scenes" folder.
3. In the Hierarchy, expand the "Networking" GameObject to reveal the Producer and Consumer. 
 * Assuming you are running both Unity and Isaac Sim on the same machine, set the "host" variable of the Producer and Consumer to either your IP address or `localhost`
 * Leave the ports as-is
4. Open Isaac Sim and the asociated USD stage
