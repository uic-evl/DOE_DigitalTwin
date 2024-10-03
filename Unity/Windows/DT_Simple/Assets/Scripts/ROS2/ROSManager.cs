using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSManager : MonoBehaviour
{
    public ROSConnection ros_con;

    public ArmSubscriber sub;
    public ArmPublisher pub;

    void Start()
    {
        // Establish ROS Connection
        ros_con = ROSConnection.GetOrCreateInstance();

        // Share with publisher and subscriber
        sub.m_Ros = ros_con;
        pub.m_Ros = ros_con;
    }
}
