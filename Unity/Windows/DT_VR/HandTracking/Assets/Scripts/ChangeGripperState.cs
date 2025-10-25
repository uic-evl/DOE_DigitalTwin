using UnityEngine;
using System.Collections;

public class ChangeGripperState : MonoBehaviour
{

    public GameObject rightJoint;
    public GameObject leftJoint;

    private float rightCurrent;
    private float leftCurrent;

    public bool gripperOpen = true;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.O))
        {
            Debug.Log("O Pressed");
            openGripper();
        }

        if (Input.GetKeyDown(KeyCode.C))
        {
            Debug.Log("C Pressed");
            closeGripper();
        }
    }

    public void openGripper()
    {
        if(!gripperOpen)
        {
            /*
            rightJoint.transform.Rotate(-10,0,0);
            leftJoint.transform.Rotate(10,0,0);
            */
            gripperOpen = true;
        }
    }

    public void closeGripper()
    {
        if(gripperOpen)
        {
            /*
            rightJoint.transform.Rotate(38,0,0);
            leftJoint.transform.Rotate(-38,0,0);
            */
            gripperOpen = false;
        }
    }
}
