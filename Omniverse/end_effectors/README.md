URDF File for MyArm300 Pi with Gripper, and the associated USD stage and python scripts to control in Isaac Sim. 

If you are creating a USD stage from scratch: 
1. Extract the meshes
2. Import the USD file into Isaac Sim
3. Create a Cube prim, and name it "Switch"
4. On the Switch prim, add an attribute called "JWM:Switch_state" of type "Int"
5. Create an "OmniPBR" material, and attatch it to Switch
5. Add "End_effector_control.py" to the firefighter
6. Add "EE_switch_controller.py" to Switch

You should now see the gripper open and close as you move the cube along the positive and negative x axis. 

Notes: 
* You may need to manually link all the mimic joints. You can do this under the "Mimic Joint" section in the property of a joint belonging to the gripper. 
