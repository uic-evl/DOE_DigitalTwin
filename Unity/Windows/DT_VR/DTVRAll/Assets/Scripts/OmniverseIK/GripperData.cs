/**************************************
GripperData Class
Use this class to create the string 
you wish to send to Omniverse over ZMQ. 

Sends a 0 is open, 1 if closed. 
**************************************/

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class GripperData : MonoBehaviour, IDataProducer
{
    public string messageToSend = "0";

    // Add any custom variables or object references you need to construct your message
    public bool gripperOpen = true;

    public void ToggleGripper()
    {
        if(gripperOpen)
        {
            gripperOpen = false;
            messageToSend = "1";
        }
        else
        {
            gripperOpen = true;
            messageToSend = "0";
        }
    }

    public string GetMessageString()
    {
        return messageToSend;
    }
}
