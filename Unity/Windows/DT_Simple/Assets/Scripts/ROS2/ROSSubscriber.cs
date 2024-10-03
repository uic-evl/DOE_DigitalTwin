/*
*  This is a very simple Subscriber script,  
*  useful for debugging your ROS connection
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSSubscriber : MonoBehaviour
{
    public string TopicName;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(TopicName, TopicFunction);
    }

    void TopicFunction(StringMsg MyMessage)
    {
        Debug.Log(MyMessage);
    }
}