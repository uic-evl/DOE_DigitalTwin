using System.Collections;
using System.Collections.Generic;
using System;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;

namespace PubSub
{
    public class Client : MonoBehaviour
    {
        public enum ClientStatus
        {
            Inactive,
            Activating,
            Active,
            Deactivating
        }
    
        [SerializeField] private string host;
        [SerializeField] private string port;
        private Talker _listener;
        //private Listener _listener;
        //private Talker _talker;
        private ClientStatus _clientStatus = ClientStatus.Inactive;

        public GameObject myArm;
        public GameObject myCube;

        //public ConcurrentQueue<float> _dataQueue = new ConcurrentQueue<float>();
        public ConcurrentQueue<Vector3> _dataQueue = new ConcurrentQueue<Vector3>();

        private void Start()
        {
            //_listener = new Listener(host, port, HandleMessage);
            _listener = new Talker(host, port, HandleMessage, _dataQueue);
            EventManager.Instance.onStartClient.AddListener(OnStartClient);
            EventManager.Instance.onClientStarted.AddListener(() => _clientStatus = ClientStatus.Active);
            EventManager.Instance.onStopClient.AddListener(OnStopClient);
            EventManager.Instance.onClientStopped.AddListener(() => _clientStatus = ClientStatus.Inactive);
        }

        private void Update()
        {
            if (_clientStatus == ClientStatus.Active)
            {
                // If there is no data, send some
                if(_dataQueue.IsEmpty)
                {
                    //_dataQueue.Enqueue(Time.deltaTime);
                    _dataQueue.Enqueue(myCube.transform.position);
                }

                // Print message
                _listener.DigestMessage();
            }
        }

        private void OnDestroy()
        {
            if (_clientStatus != ClientStatus.Inactive)
                OnStopClient();
        }

        private void HandleMessage(string message)
        {
            Debug.Log(message);
            //myArm.GetComponent<ArmZMQ>().message = message;
        }

        private void OnStartClient()
        {
            Debug.Log("Starting client...");
            _clientStatus = ClientStatus.Activating;
            _listener.Start();
            Debug.Log("Client started!");
        }

        private void OnStopClient()
        {
            Debug.Log("Stopping client...");
            _clientStatus = ClientStatus.Deactivating;
            _listener.Stop();
            Debug.Log("Client stopped!");
        } 
    }
}