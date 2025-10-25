/**************************************
ArmControllerOmniverseZMQ

Controls a URDF import of an arm from 
Omniverse commands over ZMQ. Must be 
paired with a ZMQ Consumer. 
**************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ArmControllerOmniverseZMQ : MonoBehaviour
{
    public bool convertFromRadians;

    public Consumer consumer;

    // Add the joints in the order they are recieved from Omniverse
    public ArticulationBody[] JointArticulationBodies;
    public float[] myAngles;

    private int NumRobotJoints;

    [SerializeField]
    GameObject myArm;

    private string[] data;

    void Start()
    {
        Debug.Log("Initializing");
        NumRobotJoints = JointArticulationBodies.Length;
        myAngles = new float[NumRobotJoints];
    }

    void Update()
    {
        //Debug.Log(consumer.myMessage);
        if(consumer.consumerActive)
        {
            UpdateJoints(consumer.myMessage);
        }
    }

    // Update the joint angle by setting the
    // xDrive.Target of each ArticulationBody
    public void UpdateJointAngle(float cmd, int joint)
    {
        var angle = 0.0f;


        if(convertFromRadians)
        {
            angle = cmd * Mathf.Rad2Deg;
            //data[joint] = angle.ToString(); //Debugging 
        }
        else
        {   
            angle = cmd;
        }

        var jointXDrive = JointArticulationBodies[joint].xDrive;
        jointXDrive.target = angle;
        JointArticulationBodies[joint].xDrive = jointXDrive;

        //myAngles[joint] = angle;
    }

    public void UpdateJoints(string message)
    {
        message = message.Replace("[", "").Replace("]", "");

        //string[] data = message.Split(",");
        data = message.Split(",");

        for(int i = 0; i < NumRobotJoints; i++)
        {
            data[i] = data[i].Replace("'", "");
            data[i] = data[i].Replace("b", "");
            UpdateJointAngle(float.Parse(data[i]), i);
            
            //Debug.Log("Joint " + i.ToString() + " has " + data[i].ToString());
        }
    }
}