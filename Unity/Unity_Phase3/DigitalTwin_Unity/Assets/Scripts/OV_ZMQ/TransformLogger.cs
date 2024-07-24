using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;

public class TransformLogger : MonoBehaviour
{
    private DateTime currentTime;
    private int frameNumber;

    public GameObject[] robotObjects;

    private Vector3[] positions;
    private Vector3[] rotations; 

    private string data;

    private string basePath = "C:/Users/halle/OneDrive/Documents/Hal/2024-vr-dt/digital-twin/Unity/Unity_Phase3/DigitalTwin_Unity/Assets/LogFiles/";
    public string outfile;
    public StreamWriter writer;

    void Start()
    {
        // Print initialization information
        currentTime = DateTime.Now;
        frameNumber = Time.frameCount;

        positions = new Vector3[robotObjects.Length];
        rotations = new Vector3[robotObjects.Length];

        writer = new StreamWriter(basePath + outfile, true);
    }

    void Update()
    {
        // Print time, frame number
        currentTime = DateTime.Now;
        frameNumber = Time.frameCount;

        // Get position and orientation info
        for(int i = 0; i < robotObjects.Length; i++)
        {
            positions[i] = robotObjects[i].transform.position;
            rotations[i] = robotObjects[i].transform.rotation.eulerAngles;
        }

        // Print 
        //Debug.Log("At " + currentTime.ToString() + " frame " + frameNumber.ToString() + " has for joint 6:" + positions[6].ToString() + " " + rotations[6].ToString());

        // Save to file as string
        data = currentTime.ToString("yyyy-MM-dd HH:mm:ss.fff") + "," + frameNumber.ToString();

        for(int i = 0; i < robotObjects.Length; i++)
        {
            string linkData = "," + positions[i].x.ToString("F5") + " " + positions[i].y.ToString("F5") + " " + positions[i].z.ToString("F5") + "," + rotations[i].x.ToString("F5") + " " + rotations[i].y.ToString("F5") + " " + rotations[i].z.ToString("F5");
            data += linkData;
        }

        writer.WriteLine(data);
    }

    void OnDestroy()
    {
        writer.Close();
    }
}
