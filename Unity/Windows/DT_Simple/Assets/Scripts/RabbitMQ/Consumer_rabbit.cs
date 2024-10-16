using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
using TMPro;

public class Consumer_rabbit : MonoBehaviour
{

    private IConnection connection;
    private IModel channel;
    public EventingBasicConsumer consumer;
    public string queueName;


    public bool consumerActive = false;

    //Host and port to connect to 
    [SerializeField] private string host;
    [SerializeField] private string port;

    //public TMP_Text consumerMessage;

    public GameObject myArm;
    //public GameObject myCube;

    public Thread consumerThread;
    public ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();


    // Start is called before the first frame update
    void Start()
    {

        try
        {
            var factory = new ConnectionFactory
            {
                HostName = "10.0.0.119",
                Port = 5672,
                UserName = "hema",
                Password = "hema"
            };
            connection = factory.CreateConnection();
            channel = connection.CreateModel();

            channel.ExchangeDeclare(exchange: "joints_str", type: ExchangeType.Fanout);

            // declare a server-named queue
            queueName = channel.QueueDeclare().QueueName;
            Debug.Log("queueName" + queueName);
            channel.QueueBind(queue: "joints_str",
                              exchange: "joints_str",
                              routingKey: "joints_str");

            Debug.Log(" [*] Waiting for logs.");
            consumer = new EventingBasicConsumer(channel);

            consumer.Received += (model, ea) =>
            {
                byte[] body = ea.Body.ToArray();
                var message = Encoding.UTF8.GetString(body);
                // Debug.Log("[x] {message} " + message);
                messageQueue.Enqueue(message);
            };


            channel.BasicConsume(queue: "joints_str",
                                noAck: false,
                                consumer: consumer);

        }
        catch (Exception e)
        {
            Debug.Log("failled " + e);
            //  Block of code to handle errors
        }
    }

    // Update is called once per frame
    void Update()
    {
        //ebug.Log("channel: "+channel);
        Debug.Log("in update func: "+ messageQueue.IsEmpty);

        getMessage();




    }
    private void HandleMessage(string message)
    {
     //   consumerMessage.text = message;
        // Debug.Log(message);
        myArm.GetComponent<ArmZMQ>().message = message;
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