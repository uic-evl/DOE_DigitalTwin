using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ArmZMQ : MonoBehaviour
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

    //ROSConnection m_Ros;

    public string message;

    void Start()
    {
        Debug.Log("Initializing");
        /*
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();*/

        // Create array for articulation bodies of each joint
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        // Get the articulationbody for each joint
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_Arm.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
        }

        // Subscribe to joint_states topic
        //m_Ros.Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
    }

    void Update()
    {
        //Debug.Log(message);
        UpdateJoints(message);
    }

    // Update the joint angle by setting the
    // xDrive.Target of each Articulationbody
    public void UpdateJointAngle(float cmd, int joint)
    {
        Debug.Log("Joint " + joint.ToString() + " has " + cmd.ToString());
        //var angle = (float)cmd * Mathf.Rad2Deg;
        var jointXDrive = m_JointArticulationBodies[joint].xDrive;
        jointXDrive.target = cmd;
        m_JointArticulationBodies[joint].xDrive = jointXDrive;
    }

    public void UpdateJoints(string message)
    {
        //Debug.Log(message.position[1]);
        for(int i = 0; i < k_NumRobotJoints; i++)
        {
            UpdateJointAngle(message[i + 1], i);
        }
    }

}