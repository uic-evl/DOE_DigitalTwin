/**************************************
Producer Class
A producer starts a Talker thread
that will send messages on 
a specific host/port
**************************************/

using System.Collections;
using System.Collections.Generic;
using System;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;


public class Producer : MonoBehaviour
{
    public bool producerActive = false;
    public bool initSent = false;

    //Host and port to bind socket to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    //public GameObject myArm;
    public GameObject myCube;

    public Transform RobotOrgin;
    public Transform TableOrigin;
    public Transform ArmOrigin;

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    //public ConcurrentQueue<float> dataQueue = new ConcurrentQueue<float>();

    public ConcurrentQueue<(Vector3 position, Vector3 rotation)> dataQueue = new ConcurrentQueue<(Vector3, Vector3)>();

    private void Start()
    {
        EventManager.myEvents.onStartProducer.AddListener(OnStartProducer);
        EventManager.myEvents.onStopProducer.AddListener(OnStopProducer);
    }

    private void Update()
    {
        if (producerActive)
        {
            if(dataQueue.IsEmpty && myCube.transform.hasChanged)
            {
                //dataQueue.Enqueue(Time.deltaTime);
                Vector3 rotation = myCube.transform.rotation.eulerAngles;
                dataQueue.Enqueue(((myCube.transform.position - RobotOrgin.localPosition - TableOrigin.localPosition - ArmOrigin.localPosition) * 100, rotation));
                //dataQueue.Enqueue(myCube.transform.localPosition);
                //Debug.Log("Hello from Enqueue");
                //Debug.Log(myCube.transform.position - RobotOrgin.localPosition - TableOrigin.localPosition);
                myCube.transform.hasChanged = false;
            }

            // Print message
            getMessage();
        }
    }

    private void OnDestroy()
    {
        if (producerActive)
            OnStopProducer();
    }

    private void HandleMessage(string message)
    {
        //Debug.Log("Producer: " + message);
        //myArm.GetComponent<ArmZMQ>().message = message;
    }

    private void OnStartProducer()
    {
        Debug.Log("Starting Producer...");
        startThread();
        Debug.Log("Producer started!");
    }

    private void OnStopProducer()
    {
        Debug.Log("Stopping Producer...");
        stopThread();
        Debug.Log("Producer stopped!");
    } 

    public void startThread()
    {
        Debug.Log("Start producer thread");
        producerThread = new Thread(threadWork);
        producerThread.Start();
        producerActive = true;
    }

    public void stopThread()
    {
        producerActive = false;
        producerThread?.Join();
        producerThread = null;
    }

    private void threadWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var pubSocket = new PublisherSocket())
        {
            pubSocket.Options.SendHighWatermark = 1000;
            pubSocket.Bind($"tcp://*:{port}");

            //Form message
            while (producerActive)
            {
                // If there is data in the queue, get that data
                while(!dataQueue.IsEmpty)
                {
                    if (dataQueue.TryDequeue(out var item))
                    {
                        // Create a multipart message
                        var message = new NetMQMessage();
                        message.Append("Position");
                        message.Append(item.position.ToString("F4"));
                        message.Append("Rotation");
                        message.Append(item.rotation.ToString("F4"));

                        // Send the multipart message
                        pubSocket.SendMultipartMessage(message);
                        Debug.Log($"Sent Position: {item.position}, Rotation: {item.rotation}");
                    }
                    else
                    {
                        break;
                    }
                }
            }
            pubSocket.Close();
        }
        NetMQConfig.Cleanup();
    }

    private void getMessage()
    {
        while (!messageQueue.IsEmpty)
        {
            if (messageQueue.TryDequeue(out var message))
            {
                HandleMessage(message);
            }
            else
                break;
        }
    }
}