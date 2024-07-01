/*
*  Subscribe to Elephant Robotics "/joint_states" topic  
*  and update the URDF arm visualization. 
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ArmSubscriber : MonoBehaviour
{
    // Each link name
    public static readonly string[] LinkNames =
    {   
        "base/joint1",
        "base/joint1/joint2",
        "base/joint1/joint2/joint3",
        "base/joint1/joint2/joint3/joint4",
        "base/joint1/joint2/joint3/joint4/joint5",
        "base/joint1/joint2/joint3/joint4/joint5/joint6",
        "base/joint1/joint2/joint3/joint4/joint5/joint6/joint7"
    };

    // Hardcoded variables
    const int k_NumRobotJoints = 7;

    [SerializeField]
    GameObject m_Arm;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    public ROSConnection m_Ros;

    void Awake()
    {
        // Create array for articulation bodies of each joint
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        // Get the articulationbody for each joint
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_Arm.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
        }
    }

    // Update the joint angle by setting the
    // xDrive.Target of each Articulationbody
    public void UpdateJointAngle(double cmd, int joint)
    {
        var angle = (float)cmd * Mathf.Rad2Deg;
        var jointXDrive = m_JointArticulationBodies[joint].xDrive;
        jointXDrive.target = angle;
        m_JointArticulationBodies[joint].xDrive = jointXDrive;
    }

    public void UpdateJoints(JointStateMsg message)
    {
        for(int i = 0; i < k_NumRobotJoints; i++)
        {
            UpdateJointAngle(message.position[i], i);
        }
    }

    public void Sub()
    {
        // Subscribe to joint_states topic
        m_Ros.Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
    }

    public void Unsub()
    {
        // Stop subscribing
        m_Ros.Unsubscribe("/joint_states");
    }

}