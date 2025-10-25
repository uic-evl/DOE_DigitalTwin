/**************************************
Producer Class
A producer starts a Talker thread
that will send messages on 
a specific host/port
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
    public bool initSent = false;

    // Host and port to bind socket to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    // Timer
    private float startTime;
    private float currentTime;
    private float endTime;
    public float delay;

    public bool isWait = false;

    // Public GameObject myArm;
    public GameObject myCube;

    public Transform RobotOrgin;
    public Transform ArmOrigin;

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
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
            /*
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
            */

            if(dataQueue.IsEmpty && myCube.transform.hasChanged)
            {
                // Vector3 position = myCube.transform.position - RobotOrgin.localPosition - ArmOrigin.localPosition;
                Vector3 rotation = myCube.transform.rotation.eulerAngles;

                // Debug.Log($"Enqueuing Position: {position}, Rotation: {rotation}");

                dataQueue.Enqueue((myCube.transform.position, rotation));

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
                        message.Append(item.position.ToString("F5"));
                        message.Append("Rotation");
                        message.Append(item.rotation.ToString("F5"));

                        // Send the multipart message
                        pubSocket.SendMultipartMessage(message);
                        //pubSocket.SendFrame("Hello from Unity");
                        //Debug.Log($"Sent Position: {item.position}, Rotation: {item.rotation}");
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
