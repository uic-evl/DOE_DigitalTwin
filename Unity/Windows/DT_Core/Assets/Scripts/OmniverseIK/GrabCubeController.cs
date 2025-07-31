using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class GrabCubeController : MonoBehaviour
{

    public Consumer consumer;
    public string[] data;

    public Vector3 newPosition;
    public Vector3 newRotation;

    public Vector3 posOffset;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(consumer.consumerActive)
        {
            //Debug.Log(consumer.myMessage);
            ProcessMessage(consumer.myMessage);
        }
    }

    public void ProcessMessage(string message)
    {
        data = message.Split(",");

        //Vector3 newPosition = new Vector3(float.Parse(data[0]), float.Parse(data[1]), float.Parse(data[2]));
        //Vector3 newRotation = new Vector3(float.Parse(data[3]), float.Parse(data[4]), float.Parse(data[5]));

        //OLD 
        //newPosition = new Vector3(float.Parse(data[1]) * -1.0f, float.Parse(data[2]), float.Parse(data[0]) * -1.0f);
        //newRotation = new Vector3(float.Parse(data[4]) * -1.0f, float.Parse(data[5]), float.Parse(data[3])* -1.0f);
        
        newPosition = new Vector3(float.Parse(data[0]) * -1.0f, float.Parse(data[2]), float.Parse(data[1])* -1.0f);
        newRotation = new Vector3(float.Parse(data[3]), float.Parse(data[5]) * -1.0f, float.Parse(data[4]));

        //newPosition = new Vector3(float.Parse(data[0]), float.Parse(data[1]), float.Parse(data[2]));
        //newRotation = new Vector3(float.Parse(data[3]), float.Parse(data[4]), float.Parse(data[5]));

        gameObject.transform.position = newPosition + posOffset;
        gameObject.transform.eulerAngles = newRotation;
    }
}
