# Getting Started with Omniverse

For both the Omniverse-Unity and Omniverse-Only configuration, we use Omniverse Isaac Sim for robot simulation, specifically, Inverse Kinematics control. This guide walks through how to set up an Isaac Sim scene to control the **ER MyArm300**. 

Steps:
1. Inverse Kinematics
2. Communication with Hardware
3. Connecting to Unity
4. Adding a Gripper 

## Step 1: Inverse Kinematics

1. Open a new stage in Isaac Sim
2. Open "Isaac Utils > Workflows > URDF Importer"
3. Under "Input", select the URDF for the arm found in this repository at `Robot_Files/ER_MyArm300/description/myarm_300_pi.urdf`
   * Make sure "Create Physics Scene" and "Parse Mimic Joint" are checked. 
4. Create a Ground Plane under "Create > Physics > Ground Plane"
5. Add "vr_input.py" to the "firefighter" object 
   * Update the `PATH_BASE` variable to the `DOE_DigitalTwin` folder of this directory wherever it is saved on your machine. 
   * In the `on_init` function...
     * Change `self.ee_name` to `joint7`
     * Change `robot_description_path` to `/Robot_Files/ER_MyArm300/description/rd_myarm.yaml`
       * Make sure to leave the "PATH_BASE" variable in the string
     * Change `urdf_path` to `/Robot_Files/ER_MyArm300/description/myarm_300_pi.urdf`
6. Create a new empty Xform under "firefighter" named "Target"
   * Create a Cube inside of Target and scale it down to roughly the same size as the end joint. 
     * Make sure the Cube's coordinates are (0,0,0). 
7. Save your USD stage to your disk. Keep track of this file location for later. 

You should now be able to press play and move the Target (make sure you are moving the Target Xform and not the cube) to see the arm move. If the Target is at an invalid location for the IK calculation, the console should print "IK failed".

## Step 2: Communication with Hardware

Now that we can simulate the arm in Isaac Sim, we want to send the results of this simulation to the physical arm to move it. This can be accomplished using either ZMQ or ROS. 

For either method, make sure your computer and the arm are on the same wifi network. 

### ZMQ

For ZMQ communication, you will need to add a new Python script to our USD stage, as well as create and run a script on the ER MyArm. 

In Isaac Sim, add another Python Script Asset to "firefighter". You will add the "send_joints.py" file from this repository. 

On the arm, find or download the "link_v2.py" script. You can find it in this repository in "/Robot_Files/ER_MyArm300/scripts". 
* On line 36, update the IP address to be the IP of the machine running Isaac Sim. Leave the port as it is. 

Press play in Isaac Sim, and run the python script on the arm. You should be able to move the Target in Isaac Sim to move the physical arm. 

### ROS

ROS control only works on operating systems where ROS is supported, such as Windows 10 and Ubuntu. If you are on Windows 11, you can stick with ZMQ communication. 

If you haven't already, you'll need to install ROS on your machine. See [our instructions](https://github.com/uic-evl/DOE_DigitalTwin/wiki/ROS-Installation).

In Isaac Sim: 
1. Open the Extensions window under "Windows > Extensions".
2. Search for "ROS", find and enable the "ROS2 Bridge" Extension. 
3. In the "Property" panel, make sure the "Articulation Root" is on "firefighter" and not "root_joint"
4. Add "Isaac Utils > Common Omnigraphs > ROS2 JointStates"
   * Select "firefighter" as the Target Prim
   * Select "Publisher"
5. When you press play, Isaac Sim will publish the joint angles. You can check this by running `ros2 topic list` in a terminal and checking for `/joint_states`
 
On the arm:
1. Open a ROS2 terminal
2. `cd colcon_ws`
3. `source install/setup.bash`
4. `ros2 run myarm_300 slider_control`

If Isaac Sim is running, the arm should now match its position.




