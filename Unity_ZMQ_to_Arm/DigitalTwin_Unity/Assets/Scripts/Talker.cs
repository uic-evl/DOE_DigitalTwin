using UnityEngine;
using System.Collections;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class Talker
{
    /*
    private Thread _clientThread;
    private readonly string _host;
    private readonly string _port;
    private readonly Action<string> _messageCallback;
    public Action<float> _messageUpdate;
    private bool _clientCancelled;

    public float myValue = 11;

    public ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();
    
    public ConcurrentQueue<float> _dataQueue;
    //public ConcurrentQueue<Vector3> _dataQueue;

    public Talker(string host, string port, Action<string> messageCallback, ConcurrentQueue<float> dataQueue)
    {
        _host = host;
        _port = port;
        _messageCallback = messageCallback;
        _dataQueue = dataQueue;
    }   

    public void Start()
    {
        _clientCancelled = false;
        _clientThread = new Thread(TalkerWork);
        _clientThread.Start();
        EventManager.Instance.onClientStarted.Invoke();
    }

    public void Stop()
    {
        _clientCancelled = true;
        _clientThread?.Join();
        _clientThread = null;
        EventManager.Instance.onClientStopped.Invoke();
    }

    private void TalkerWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var pubSocket = new PublisherSocket())
        {
            pubSocket.Options.ReceiveHighWatermark = 1000;
            pubSocket.Bind($"tcp://*:{_port}");

            //Form message
            string message = "temp";

            while (!_clientCancelled)
            {
                Debug.Log("Saying Hello");
                //string message = myCube.transform.position.ToString();

                //If there is data in the queue, get that data
                if(!_dataQueue.IsEmpty)
                {
                    //Send message
                    if (_messageQueue.TryPeek(out var item))
                    {
                        pubSocket.SendFrame(item);
                    }

                    if (_dataQueue.TryDequeue(out var data))
                    {
                        message = data.ToString(); 
                    }
                    else
                        break;
                }

                //Add message to queue for printing
                _messageQueue.Enqueue(message);
            }
            pubSocket.Close();
        }
        NetMQConfig.Cleanup();
    }

    public void DigestMessage()
    {
        while (!_messageQueue.IsEmpty)
        {
            if (_messageQueue.TryDequeue(out var message))
            {
                _messageCallback(message);
            }
            else
                break;
        }
    }*/
}
