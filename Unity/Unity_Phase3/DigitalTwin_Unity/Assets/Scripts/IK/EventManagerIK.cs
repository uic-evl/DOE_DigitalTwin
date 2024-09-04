/**************************************
EventManager Class
The Event Manager handles events for
stopping and starting producers and
consumers across threads
**************************************/
using UnityEngine;
using UnityEngine.Events;

public class EventManagerIK : MonoBehaviour
{
    public UnityEvent onStartProducer;
    public UnityEvent onStopProducer;

    public static EventManagerIK myEvents;

    private void Awake()
    {
        if (myEvents == null)
        {
            myEvents = this;

            onStartProducer = new UnityEvent();
            onStopProducer = new UnityEvent();
        }
        else
            Destroy(this);
    }
}
