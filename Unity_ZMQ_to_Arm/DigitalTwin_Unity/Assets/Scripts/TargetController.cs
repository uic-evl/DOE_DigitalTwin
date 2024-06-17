using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetController : MonoBehaviour
{

    public Producer producer;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //Increase X
        if(Input.GetKeyDown(KeyCode.U))
        {
            gameObject.transform.Translate(0.01f, 0, 0);
        }

        //Decrease X
        if(Input.GetKeyDown(KeyCode.I))
        {
            gameObject.transform.Translate(-0.01f, 0, 0);
        }

        //Increase Y
        if(Input.GetKeyDown(KeyCode.J))
        {
            gameObject.transform.Translate(0, 0.01f, 0);
        }

        //Decrease Y
        if(Input.GetKeyDown(KeyCode.K))
        {
            gameObject.transform.Translate(0, -0.01f, 0);
        }

        //Increase Z
        if(Input.GetKeyDown(KeyCode.N))
        {
            gameObject.transform.Translate(0, 0, 0.01f);
        }

        //Decrease Z
        if(Input.GetKeyDown(KeyCode.M))
        {
            gameObject.transform.Translate(0, 0, -0.01f);
        }
    }
}
