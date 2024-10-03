/**************************************
IKFailChecker Class
A consumer starts a thread that will
listen (subscribe) on a specific 
host/port for a message
**************************************/

using System.Collections;
using System.Collections.Generic;
using System;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;
using TMPro;
using NetMQ;
using NetMQ.Sockets;

public class IKFailChecker : MonoBehaviour
{
    public bool consumerActive = false;
    
    //Host and port to connect to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    public GameObject myCube;

    public Material IKGood;
    public Material IKBad;

    public Thread consumerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    private void Start()
    {
        EventManager.myEvents.onStartConsumer.AddListener(OnStartConsumer);
        EventManager.myEvents.onStopConsumer.AddListener(OnStopConsumer);
    }

    private void Update()
    {
        if (consumerActive)
        {
            // Print message
            getMessage();
        }
    }

    private void OnDestroy()
    {
        if (consumerActive)
            OnStopConsumer();
    }

    private void HandleMessage(string message)
    {
        //consumerMessage.text = message;
        Debug.Log(message);
        //myArm.GetComponent<ArmZMQ>().message = message;
        // Change cube color
        // 1 = IK has failed
        if(message == "0")
        {
            myCube.GetComponent<MeshRenderer>().material = IKGood;
        }
        else if(message == "1")
        {
            myCube.GetComponent<MeshRenderer>().material = IKBad;
        }
    }

    private void OnStartConsumer()
    {
        Debug.Log("Starting Consumer...");
        startThread();
        Debug.Log("Consumer started!");
    }

    private void OnStopConsumer()
    {
        Debug.Log("Stopping Consumer...");
        stopThread();
        Debug.Log("Consumer stopped!");
    } 

    public void startThread()
    {
        consumerThread = new Thread(threadWork);
        consumerThread.Start();
        consumerActive = true;
    }

    public void stopThread()
    {
        consumerActive = false;
        consumerThread?.Join();
        consumerThread = null;
    }

    private void threadWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var subSocket = new SubscriberSocket())
        {
            subSocket.Options.ReceiveHighWatermark = 1000;
            subSocket.Connect($"tcp://{host}:{port}");
            subSocket.SubscribeToAnyTopic();
            while (consumerActive)
            {
                if (!subSocket.TryReceiveFrameString(out var message)) continue;
                messageQueue.Enqueue(message);
            }
            subSocket.Close();
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