/**************************************
TargetData Class
Use this class to create the string 
you wish to send to Omniverse over ZMQ. 

"Target" here referes specifically to 
the Inverse Kinematics Target. 
**************************************/

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class TargetData : MonoBehaviour, IDataProducer
{
    public string messageToSend;

    // Add any custom variables or object references you need to construct your message
    // Public GameObject myArm;
    public GameObject myCube;

    public Transform RobotOrgin;
    public Transform ArmOrigin;

    // Add custom code here to format your message however you need
    void Update()
    {
        Vector3 position = myCube.transform.position - RobotOrgin.localPosition - ArmOrigin.localPosition;
        Vector3 rotation = myCube.transform.rotation.eulerAngles;

        //messageToSend = myCube.transform.position.ToString("F5") + "," + rotation.ToString("F5");
        messageToSend = position.ToString("F5") + "," + rotation.ToString("F5");
        //Debug.Log(messageToSend);
    }

    public string GetMessageString()
    {
        return messageToSend;
    }
}
