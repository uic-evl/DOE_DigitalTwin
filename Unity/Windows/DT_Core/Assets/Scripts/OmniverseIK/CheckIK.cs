using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKCheck : MonoBehaviour
{
    public Material IKGood;
    public Material IKBad;

    public GameObject myCube;

    public Consumer ikChecker;

    // Update is called once per frame
    void Update()
    {
        if(ikChecker.myMessage == "0")
        {
            myCube.GetComponent<MeshRenderer>().material = IKGood;
        }
        else if(ikChecker.myMessage == "1")
        {
            myCube.GetComponent<MeshRenderer>().material = IKBad;
        }
    }
}
