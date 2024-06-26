/**************************************
EventManager Class
The Event Manager handles events for
stopping and starting producers and
consumers across threads
**************************************/
using UnityEngine;
using UnityEngine.Events;

public class EventManager : MonoBehaviour
{
    public UnityEvent onStartConsumer;
    public UnityEvent onStopConsumer;

    public UnityEvent onStartProducer;
    public UnityEvent onStopProducer;

    public static EventManager myEvents;

    private void Awake()
    {
        if (myEvents == null)
        {
            myEvents = this;
        
            onStartConsumer = new UnityEvent();
            onStopConsumer = new UnityEvent();

            onStartProducer = new UnityEvent();
            onStopProducer = new UnityEvent();
        }
        else
            Destroy(this);
    }
}
