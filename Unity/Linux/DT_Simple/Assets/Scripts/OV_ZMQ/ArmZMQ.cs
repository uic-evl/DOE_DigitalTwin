/**************************************
ArmZMQ Class
Receive message from Consumer, then 
parse out the joint angles and apply 
the the URDF-based ArticulationBodies
**************************************/

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

    public string message;

    void Start()
    {
        Debug.Log("Initializing");

        // Create array for articulation bodies of each joint
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        // Get the ArticulationBody for each joint
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_Arm.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
        }
    }

    void Update()
    {
        //Debug.Log(message);
        UpdateJoints(message);
    }

    // Update the joint angle by setting the
    // xDrive.Target of each ArticulationBody
    public void UpdateJointAngle(float cmd, int joint)
    {
        var angle = cmd * Mathf.Rad2Deg;
        var jointXDrive = m_JointArticulationBodies[joint].xDrive;
        jointXDrive.target = angle;
        m_JointArticulationBodies[joint].xDrive = jointXDrive;
    }

    public void UpdateJoints(string message)
    {
        message = message.Replace("[", "").Replace("]", "");

        string[] data = message.Split(',');

        for(int i = 0; i < k_NumRobotJoints; i++)
        {
            data[i] = data[i].Replace("'", "");
            data[i] = data[i].Replace("b", "");
            //Debug.Log("Joint " + i.ToString() + " has " + data[i].ToString());
            UpdateJointAngle(float.Parse(data[i]), i);
        }
    }
}
