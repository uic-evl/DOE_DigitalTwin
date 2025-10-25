/*
*  Publish joint psoitions to Elephant Robotics "/joint_states" topic  
*  using the Unity URDF controller
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using Unity.Mathematics;

public class ArmPublisher : MonoBehaviour
{
    public GetJointAngles jointData;

    // Each joint name from URDF
    public static readonly string[] JointNames =
    {   
        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint7_to_joint6"
    };

    // Hardcoded variables
    const int k_NumRobotJoints = 7;

    [SerializeField]
    GameObject m_Arm;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Variables
    public ROSConnection m_Ros;
    public JointStateMsg jointMsg;
    double[] jointAnglesROS;

    // Null for message data
    public HeaderMsg header;
    public double[] velocity;
    public double[] effort; 

    // Publishing variables
    public bool isPublish = false;
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;

    void Awake()
    {
        //Initialize ROS message
        jointAnglesROS = new double[k_NumRobotJoints];
        header = new HeaderMsg();
        velocity = new double[k_NumRobotJoints];
        effort = new double[k_NumRobotJoints];
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        
        // Get and publish joint state
        if(isPublish && timeElapsed > publishMessageFrequency)
        {
            GetJoints();

            m_Ros.Publish("/joint_states", jointMsg);

            timeElapsed = 0;
        }
    }

    // Get the joint angle from the
    // xDrive.Target of each Articulationbody
    public void GetJointAngle(int indx)
    {
        float angle = jointData.jointAngles[indx];

        if(float.IsNaN(angle))
        {
            angle = 0.0f;
        }

        if(indx % 2 == 0)
        {
            angle *= -1.0f;
        }

        if(indx == 3)
        {
            angle *= -1.0f;
        }

        if(indx == 5)
        {
            angle *= -1.0f;
        }

        jointAnglesROS[indx] = angle * Mathf.Deg2Rad;
    }

    public void GetJoints()
    {
        for(int i = 0; i < k_NumRobotJoints; i++)
        {
            GetJointAngle(i);
        }

        //Create as JointStateMsg object
        jointMsg = new JointStateMsg(header, JointNames, jointAnglesROS, velocity, effort);
    }

    public void Pub()
    {
        // Publish
        m_Ros.RegisterPublisher<JointStateMsg>("/joint_states");
        isPublish = true;
    }

    public void StopPub()
    {
        // Stop Publishing
        isPublish = false;
    }

}