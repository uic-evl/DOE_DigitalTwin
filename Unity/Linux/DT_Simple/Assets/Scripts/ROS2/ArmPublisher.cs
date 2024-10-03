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

public class ArmPublisher : MonoBehaviour
{
    // Each link name from Unity Hierarchy
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
    
        // Create array for articulation bodies of each joint
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        jointAnglesROS = new double[k_NumRobotJoints];

        // Get the articulationbody for each joint
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_Arm.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
        }

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
        var angle =  m_JointArticulationBodies[indx].xDrive.target * Mathf.Deg2Rad;
        jointAnglesROS[indx] = angle;
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
