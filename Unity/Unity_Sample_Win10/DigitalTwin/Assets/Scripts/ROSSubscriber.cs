using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class ROSSubscriber : MonoBehaviour
{
    public string TopicName;
    public Vector3Msg accel;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<ImuMsg>(TopicName, TopicFunction);
    }

    void TopicFunction(ImuMsg MyMessage)
    {
        Debug.Log("Hello");
        Debug.Log(MyMessage.linear_acceleration);
        accel = MyMessage.linear_acceleration;
    }
}