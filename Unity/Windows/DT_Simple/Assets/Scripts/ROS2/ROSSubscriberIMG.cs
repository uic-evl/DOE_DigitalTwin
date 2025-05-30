/*
*  This is a very simple Subscriber script,  
*  useful for debugging your ROS connection
*/

using System;
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ROSSubscriberIMG : MonoBehaviour
{
    private string TopicName = "image_raw";

    public ROSConnection m_Ros;

    public bool imgInit = false;
    public Texture2D outTexture;
    public Renderer display;

    public void Sub()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(TopicName, LoadImage);
    }

    void LoadImage(ImageMsg IMGmsg)
    {

        // I am asuming the image encoding is "rgb8"
        if(imgInit == false)
        {
            imgInit = true;
            
            outTexture = new Texture2D((int)IMGmsg.width, (int)IMGmsg.height, TextureFormat.RGB24, false);
            display.material.mainTexture = outTexture;

        }
        else
        {
            // Apply to texture
            outTexture.LoadRawTextureData(IMGmsg.data);
            outTexture.Apply();
        }
    }

    public void Unsub()
    {
        // Stop subscribing
        m_Ros.Unsubscribe(TopicName);
    }
}