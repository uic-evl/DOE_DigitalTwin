/**************************************
PointData Class
Use this class to create the string 
you wish to send to Omniverse over ZMQ. 

Sends XYZ Cartesian point
**************************************/

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class PointData : MonoBehaviour, IDataProducer
{
    public string messageToSend;

    // Add any custom variables or object references you need to construct your message


    // Add custom code here to format your message however you need
    void Update()
    {
        Vector3 position = gameObject.transform.position;
        messageToSend = position.ToString("F10");
    }

    public string GetMessageString()
    {
        return messageToSend;
    }
}