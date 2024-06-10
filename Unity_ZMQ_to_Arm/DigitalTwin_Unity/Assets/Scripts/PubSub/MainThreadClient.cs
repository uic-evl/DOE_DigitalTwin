using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;


public class MainThreadClient : MonoBehaviour
{

    public GameObject myCube;
    
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        AsyncIO.ForceDotNet.Force();
        using (var pubSocket = new PublisherSocket())
        {
            pubSocket.Options.ReceiveHighWatermark = 1000;
            pubSocket.Connect($"tcp://localhost:12346");

            Debug.Log("Unity is sending");
            string message = myCube.transform.position.ToString();
            pubSocket.SendFrame("Unity Hello: " + message);

        }
        NetMQConfig.Cleanup();
    }

    void OnDestroy()
    {

    }
}
