/**************************************
ArmZMQ Class
Receive message from Consumer, then 
parse out the joint angles and apply 
the the URDF-based ArticulationBodies
**************************************/
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;
using NetMQ;
using NetMQ.Sockets;

public class SendJointsZMQ : MonoBehaviour
{
    public GetJointAngles jointData;
    public GripperController gripperController;

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

    public string message;

    private void Update()
    {
        // Get a list of the joint angles and format it into a string
        //message = jointData.jointAngles.ToString();
        message = jointData.jointAngles[0].ToString() + "," + jointData.jointAngles[1].ToString() + "," + jointData.jointAngles[2].ToString() + "," + jointData.jointAngles[3].ToString() + "," + jointData.jointAngles[4].ToString() + "," + jointData.jointAngles[5].ToString() + "," + jointData.jointAngles[6] + "," + gripperController.GetGripperState().ToString();

        // Send message to the arm
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

            //if(dataQueue.IsEmpty && isWait == false)
            if(isWait == false)
            {
                dataQueue.Enqueue((message));
                Debug.Log("Enqueue");
            }
        }
    }

    public void OnStartProducer()
    {
        Debug.Log("Starting Producer...");
        startThread();
        Debug.Log("Producer started!");
    }

    public void startThread()
    {
        Debug.Log("Start producer thread");
        producerThread = new Thread(threadWork);
        producerThread.Start();
        producerActive = true;
    }

    public void OnStopProducer()
    {
        Debug.Log("Stopping Producer...");
        stopThread();
        Debug.Log("Producer stopped!");
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
                        var msg = new NetMQMessage();
                        msg.Append(item);
                        pubSocket.SendMultipartMessage(msg);
                        //Debug.Log("Sending message");
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
}