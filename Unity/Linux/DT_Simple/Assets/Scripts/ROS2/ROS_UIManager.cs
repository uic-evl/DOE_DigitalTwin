using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROS_UIManager : MonoBehaviour
{
    public ArmSubscriber subscriber;
    public ArmPublisherCustomIK publisher;

    public void startSub()
    {
        subscriber.Sub();
    }

    public void stopSub()
    {
        subscriber.Unsub();
    }

    public void startPub()
    {
        publisher.Pub();
    }

    public void stopPub()
    {
        publisher.StopPub();
    }
}
