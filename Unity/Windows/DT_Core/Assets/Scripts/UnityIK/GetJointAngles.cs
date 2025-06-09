using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetJointAngles : MonoBehaviour
{
    public GameObject[] joints;
    public float[] jointAngles = new float[7];

    float myMin;
    float myMax;

    float myNonAngle;

    // Start is called before the first frame update
    void Start()
    {
        for(int i = 0; i < jointAngles.Length; i++)
        {
            jointAngles[i] = 0.0f;
        }
    }

    // Update is called once per frame
    void Update()
    {
        for(int i = 0; i < joints.Length; i++)
        {
            /* 
            myMin = joints[i].GetComponent<HingeJoint>().limits.min;
            myMax = joints[i].GetComponent<HingeJoint>().limits.max;

            myNonAngle = 360.0f - (myMax + Mathf.Abs(myMin));

            if(i % 2 == 0)
            {
                jointAngles[i] = joints[i].transform.eulerAngles.y;
            }
            else
            {
                jointAngles[i] = joints[i].transform.eulerAngles.x;
            }*/

            //jointAngles[i] = joints[i].transform.localEulerAngles.y;

            /*
            //Out of bounds - Positive
            if(jointAngles[i] > myMax)
            {
                //jointAngles[i] = jointAngles[i] - myMax - (myNonAngle/2);
                jointAngles[i] = jointAngles[i] - 360;
            }
            //Out of bounds - Negative
            else if(jointAngles[i] < myMin)
            {
                //jointAngles[i] = jointAngles[i] + Mathf.Abs(myMin) + (myNonAngle/2);
                jointAngles[i] = jointAngles[i] + 360;
            }*/

            /*      
            if(i > 5)
            {
                jointAngles[i] = 0.0f;
            }*/

            jointAngles[i] = joints[i].GetComponent<HingeJoint>().angle;

            // Normalize angle by shifting 0 to myMin
            //jointAngles[i] = jointAngles[i] - myMin;
        }

        //Debug.Log(jointAngles[0].ToString() + " " + jointAngles[1].ToString() + " " + jointAngles[2].ToString() + " " + jointAngles[3].ToString() + " " + jointAngles[4].ToString() + " " + jointAngles[5].ToString() + " " + jointAngles[6]);
        //Debug.Log(jointAngles[0].ToString());
    }
}
