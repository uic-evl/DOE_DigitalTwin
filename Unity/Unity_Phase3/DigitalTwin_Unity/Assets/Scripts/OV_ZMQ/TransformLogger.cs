using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TransformLogger : MonoBehaviour
{
    private float currentTime;
    private int frameNumber;

    public GameObject[] robotObjects;

    private Vector3[] positions;
    private Vector3[] rotations; 

    void Start()
    {
        // Print initialization information
        currentTime = Time.time;
        frameNumber = Time.frameCount;

        positions = new Vector3[robotObjects.Length];
        rotations = new Vector3[robotObjects.Length];
    }

    void Update()
    {
        // Print time, frame number
        currentTime = Time.time;
        frameNumber = Time.frameCount;

        // Get position and orientation info
        for(int i = 0; i < robotObjects.Length; i++)
        {
            positions[i] = robotObjects[i].transform.position;
            rotations[i] = robotObjects[i].transform.rotation.eulerAngles;
        }

        // Print 
        Debug.Log("At " + currentTime.ToString() + " frame " + frameNumber.ToString() + " has for joint 6:" + positions[6].ToString() + " " + rotations[6].ToString());

        // Save to file

    }
}
