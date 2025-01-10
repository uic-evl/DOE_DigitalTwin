using UnityEngine;
using Oculus.Interaction.Grab;
using Oculus.Interaction.GrabAPI;
using Oculus.Interaction.Input;
using Oculus.Interaction.Throw;
using Oculus.Interaction.HandGrab;

public class TestPoseSelect : MonoBehaviour
{
    //Grabbable Object Prefab
    public GameObject grabPoint;

    //Joint to Move
    public GameObject selectedJoint;

    //Pinch Transform
    public HandGrabInteractor pinchPoint;

    public GameObject myPoint;

    private Vector3 offset = new Vector3(0.0f, 0.02f, 0.0f); //Used if setting moveable point position to the joint, instead of the pinch point

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(myPoint != null)
        {
            myPoint.transform.position = pinchPoint.PinchPoint.transform.position;
            myPoint.transform.rotation = pinchPoint.PinchPoint.transform.rotation;
        }
    }

    public void CheckSelect()
    {
        Debug.Log("Pinch Detected! Wow!!");
        
        if(myPoint == null)
        {
            //myPoint = Instantiate(grabPoint, pinchPoint.PinchPoint.transform.position, pinchPoint.PinchPoint.transform.rotation);
            myPoint = Instantiate(grabPoint, selectedJoint.transform.position, selectedJoint.transform.rotation);
            myPoint.GetComponent<FixedJoint>().connectedBody = selectedJoint.GetComponent<Rigidbody>();
        }
    }

    public void Deselect()
    {
        if(myPoint != null)
        {
            Destroy(myPoint);
        }
    }
}
