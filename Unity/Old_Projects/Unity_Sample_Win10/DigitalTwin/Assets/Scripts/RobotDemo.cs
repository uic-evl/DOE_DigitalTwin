using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotDemo : MonoBehaviour
{

    public ROSSubscriber Subscriber;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.Rotate(0,(float)Subscriber.accel.x,0);
    }
}
