using UnityEngine;
using System.Collections;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace PubSub
{
    public class Talker
    {
        private Thread _clientThread;
        private readonly string _host;
        private readonly string _port;
        private readonly Action<string> _messageCallback;
        private bool _clientCancelled;

        private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

        private GameObject myCube;

        public Talker(string host, string port, Action<string> messageCallback, GameObject clientCube)
        {
            _host = host;
            _port = port;
            _messageCallback = messageCallback;
            myCube = clientCube;
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
                pubSocket.Connect($"tcp://{_host}:{_port}");

                while (!_clientCancelled)
                {
                    //Debug.Log("Saying Hello");
                    string message = myCube.transform.position.ToString();
                    pubSocket.SendFrame(message);
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
                    _messageCallback(message);
                else
                    break;
            }
        }
    }
}