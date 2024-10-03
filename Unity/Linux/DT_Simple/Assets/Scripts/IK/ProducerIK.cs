/**************************************
ProducerIK Class
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

public class ProducerIK : MonoBehaviour
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

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    public ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();

    public IKManager ik_manager;
    public string angles_message;

    public EventManagerIK EventManagerIK;

    private void Start()
    {
        EventManagerIK.onStartProducer.AddListener(OnStartProducer);
        EventManagerIK.onStopProducer.AddListener(OnStopProducer);
    }

    public string getAnglesString(float[] angles)
    {
        string result = "";

        for(int i = 0; i < angles.Length; i++)
        {
            result += angles[i].ToString("F3") + ", ";
        }

        return result;
    }

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
                angles_message = getAnglesString(ik_manager.angles);

                Debug.Log("Enqueuing angles: " + angles_message);

                dataQueue.Enqueue(angles_message);
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
                        // Send the multipart message
                        pubSocket.SendFrame(item);
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
