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

    //Host and port to bind socket to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    //public GameObject myArm;
    //public GameObject myCube;

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    public ConcurrentQueue<float> dataQueue = new ConcurrentQueue<float>();

    //public ConcurrentQueue<Vector3> _dataQueue = new ConcurrentQueue<Vector3>();

    private void Start()
    {
        EventManager.myEvents.onStartProducer.AddListener(OnStartProducer);
        EventManager.myEvents.onStopProducer.AddListener(OnStopProducer);
    }

    private void Update()
    {
        if (producerActive)
        {
            // If there is no data, send some
            if(dataQueue.IsEmpty)
            {
                dataQueue.Enqueue(Time.deltaTime);
                //_dataQueue.Enqueue(myCube.transform.position);
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
            pubSocket.Options.ReceiveHighWatermark = 1000;
            pubSocket.Bind($"tcp://*:{port}");

            //Form message
            string message = "temp";

            while (producerActive)
            {
                Debug.Log("Saying Hello");
                //string message = myCube.transform.position.ToString();

                //If there is data in the queue, get that data
                if(!dataQueue.IsEmpty)
                {
                    //Send message
                    if (messageQueue.TryPeek(out var item))
                    {
                        pubSocket.SendFrame(item);
                    }

                    if (dataQueue.TryDequeue(out var data))
                    {
                        message = data.ToString(); 
                    }
                    else
                        break;
                }

                //Add message to queue for printing
                messageQueue.Enqueue(message);
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