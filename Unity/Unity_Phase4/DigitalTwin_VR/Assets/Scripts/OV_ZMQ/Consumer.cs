/**************************************
Consumer Class
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
using UnityEngine.UI;
using TMPro;
using NetMQ;
using NetMQ.Sockets;

public class Consumer : MonoBehaviour
{
    public bool consumerActive = false;
    
    // Host and port to connect to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    public GameObject myArm;
    // Public GameObject myCube;

    public Thread consumerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    private void Start()
    {
        //OnStartConsumer();
    }

    private void Update()
    {
        if (consumerActive)
        {
            // Print message
            getMessage();
        }
    }

    public void toggleConsumer(Toggle toggle)
    {
        if(toggle.isOn && !consumerActive)
        {
            OnStartConsumer();
        }
        else if(!toggle.isOn && consumerActive)
        {
            OnStopConsumer();
        }
    }

    private void OnDestroy()
    {
        if (consumerActive)
            OnStopConsumer();
    }

    private void HandleMessage(string message)
    {
        Debug.Log(message);
        myArm.GetComponent<GripperZMQ>().message = message;
    }

    public void OnStartConsumer()
    {
        Debug.Log("Starting Consumer...");
        startThread();
        consumerActive = true;
        Debug.Log("Consumer started!");
    }

    public void OnStopConsumer()
    {
        Debug.Log("Stopping Consumer...");
        consumerActive = false;
        stopThread();
        Debug.Log("Consumer stopped!");
    } 

    public void startThread()
    {
        consumerThread = new Thread(threadWork);
        consumerThread.Start();
    }

    public void stopThread()
    {
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
