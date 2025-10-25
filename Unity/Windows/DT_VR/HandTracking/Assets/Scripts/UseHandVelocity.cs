using UnityEngine;
using Meta.XR.Util;

public class UseHandVelocity : MonoBehaviour
{
    public GameObject RightHandAnchor;

    public Vector3 velocity;

    public Vector3 lastPosition;

    private void FixedUpdate()
    {   
        velocity = (RightHandAnchor.transform.position - lastPosition) / Time.deltaTime;
        //update previous positions
        lastPosition = RightHandAnchor.transform.position;   

        //gameObject.GetComponent<Rigidbody>().angularVelocity = velocity;
        //gameObject.GetComponent<Rigidbody>().AddForce(velocity, ForceMode.VelocityChange);
        gameObject.GetComponent<Rigidbody>().AddForce(velocity * 5000, ForceMode.Force);
   
    }
    
    //public Vector3 handVelocity;
    //public OVRInput.Controller hand;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        //handVelocity = OVRInput.GetLocalControllerVelocity(hand);
    }

    // Update is called once per frame
    void Update()
    {
        //handVelocity = OVRInput.GetLocalControllerVelocity(hand);
        //gameObject.GetComponent<Rigidbody>().linearVelocity = velocity;
    }
}
