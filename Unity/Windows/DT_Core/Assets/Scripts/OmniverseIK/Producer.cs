/**************************************
Producer Class
A producer starts a Talker thread
that will send messages on 
a specific host/port.
Requires a IDataProducer compatible class
to provide the message as a string. 
**************************************/

using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;

public class Producer : MonoBehaviour
{
    public bool producerActive = false;

    // The messageData GameObject must part of the IDataProducer interface. 
    public GameObject messageData;

    // Host and port to bind socket to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    public ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();

    // Timer
    private float startTime;
    private float currentTime;
    private float endTime;
    public float delay;

    public bool isWait = false;

    public string myName;

    private void Update()
    {
        if (producerActive)
        {
            if(isWait == false)
            {
                startTime = Time.time;
                endTime = startTime + delay;
                isWait = true;
            }

            if(Time.time >= endTime)
            {
                isWait = false;
            }

            if(dataQueue.IsEmpty && isWait == false)
            {
                if(messageData.TryGetComponent(out IDataProducer data))
                {
                    dataQueue.Enqueue(data.GetMessageString());
                }
                else
                {
                    Debug.Log("IDataProducer call failed");
                }
            }

            // Print message
            getMessage();
        }
    }

    private void OnDestroy()
    {
        NetMQConfig.Cleanup();
        /*
        if (producerActive)
            OnStopProducer();
        */
    }

    // Only used for debug output
    private void HandleMessage(string message)
    {
        //Debug.Log("Producer: " + message);
    }

    public void toggleProducer()
    {
        producerActive = !producerActive;

        if(producerActive)
        {
            OnStartProducer();
        }
        else
        {
            OnStopProducer();
        }
        
    }

    public void OnStartProducer()
    {
        Debug.Log("Starting Producer...");
        startThread();
        Debug.Log("Producer started!");
    }

    public void OnStopProducer()
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
        //producerThread?.Join();
        producerThread = null;
    }

    private void threadWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var pubSocket = new PublisherSocket())
        {
            pubSocket.Options.SendHighWatermark = 1000;
            pubSocket.Bind($"tcp://*:{port}");

            while(producerActive)
            {
                // If there is data in the queue, get that data
                while(!dataQueue.IsEmpty)
                {
                    if (dataQueue.TryDequeue(out var item))
                    {
                        // Send string
                        pubSocket.SendFrame(item.ToString());
                        //Debug.Log("Producer " + myName + ": " + item.ToString());
                    }
                    else
                    {
                        break;
                    }
                }
            }
            Debug.Log("Closing " + myName + " socket");
            pubSocket.Close();
        }
        //Debug.Log("NetMQ " + myName + " cleanup");
        //NetMQConfig.Cleanup();
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
