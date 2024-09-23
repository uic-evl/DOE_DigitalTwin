using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RabbitMQ.Client;
using RabbitMQ.Client.Events;
using System.Text;
using System.Linq;
using System;

using System.Collections.Concurrent;
using System.Threading;
using System.IO;

public class Producer_Rabbit : MonoBehaviour
{
    // Start is called before the first frame update

    private IConnection connection;
    private IModel channel;
    public EventingBasicConsumer consumer;
    public string queueName;



    public bool producerActive = false;
    public bool initSent = false;

    //Host and port to bind socket to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    //Timer
    private float startTime;
    private float currentTime;
    private float endTime;
    public float delay;

    public bool isWait = false;

    //public GameObject myArm;
    public GameObject myCube;

    public Transform RobotOrgin;
    public Transform ArmOrigin;

    public Thread producerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    public ConcurrentQueue<(Vector3 position, Vector3 rotation)> dataQueue = new ConcurrentQueue<(Vector3, Vector3)>();




    void Start()
    {

       // Debug.Log(dataQueue.IsEmpty);

        // If there is data in the queue, get that data



        //ebug.Log("channel: "+channel);

        if (dataQueue.IsEmpty )//myCube.transform.hasChanged)
        {
            //Vector3 position = myCube.transform.position - RobotOrgin.localPosition - ArmOrigin.localPosition;
            Vector3 rotation = myCube.transform.rotation.eulerAngles;

            //Debug.Log($"Enqueuing Position: {position}, Rotation: {rotation}");

            dataQueue.Enqueue((myCube.transform.position, rotation));

            myCube.transform.hasChanged = false;
        }





    }




    // Update is called once per frame

    private void getMessage()
    {
        while (!messageQueue.IsEmpty)
        {
            if (messageQueue.TryDequeue(out var message))
            {
                Debug.Log("message from rabbit mq:"+message);
            }
            else
                break;
        }
    }

    void Update()
    {
        if (dataQueue.IsEmpty)//myCube.transform.hasChanged)
        {
            //Vector3 position = myCube.transform.position - RobotOrgin.localPosition - ArmOrigin.localPosition;
            Vector3 rotation = myCube.transform.rotation.eulerAngles;

            //Debug.Log($"Enqueuing Position: {position}, Rotation: {rotation}");

            dataQueue.Enqueue((myCube.transform.position, rotation));

            myCube.transform.hasChanged = false;
        }




        while (!dataQueue.IsEmpty)
        {
            if (dataQueue.TryDequeue(out var item))
            {
                //Debug.Log("item:"+item);
                var factory = new ConnectionFactory
                {
                    HostName = "10.0.0.119",
                    Port = 5672,
                    UserName = "hema",
                    Password = "hema"
                };
                using var connection = factory.CreateConnection();
                using var channel = connection.CreateModel();

                channel.QueueDeclare(queue: "logs",
                     durable: false,
                     exclusive: false,
                     autoDelete: false,
                     arguments: null);

                var message = item.position.ToString("F3") + "," + item.rotation.ToString("F3");
                var body = Encoding.UTF8.GetBytes(message);
                var logger = new SimpleLogger("simplelog.txt");
                logger.Log(message);
                channel.BasicPublish(basicProperties: null,
                              exchange: "",
                              routingKey: "logs",
                                     body: body);
              //  Debug.Log($" [x] Sent {message}");

            }
            else
            {
                break;
            }
        }

       // getMessage();


    }
}


class SimpleLogger
{
    private readonly string _logFilePath;

    public SimpleLogger(string logFilePath)
    {
        _logFilePath = logFilePath;
    }

    public void Log(string message)
    {
        var logMessage = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss} {message}";
        File.AppendAllText(_logFilePath, logMessage + Environment.NewLine);
    }
}

