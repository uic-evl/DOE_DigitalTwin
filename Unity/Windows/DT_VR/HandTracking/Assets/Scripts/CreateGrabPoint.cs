using UnityEngine;
using Oculus.Interaction;

public class CreateGrabPoint : MonoBehaviour
{
    public GameObject GrabbablePoint;
    private GameObject myPoint;

    private GameObject myJoint;
    
    public int myJointID = 0;

    public TransformerUtils.PositionConstraints joint1PositionConstraints;
    public TransformerUtils.RotationConstraints joint1RotationConstraints;

    public TransformerUtils.PositionConstraints joint2PositionConstraints;
    public TransformerUtils.RotationConstraints joint2RotationConstraints;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void setJointID( int jointID )
    {
        myJointID = jointID;
    }

    public void SpawnGrab( GameObject selectedJoint)
    {
        myJoint = selectedJoint;

        if(myPoint == null)
        {
            //Create Point
            myPoint = Instantiate(GrabbablePoint, myJoint.transform.position, myJoint.transform.rotation);
            

            if(myJoint.tag == "LowerJoint")
            {
                //Bypass Spring
                myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<FixedJoint>().connectedBody = myJoint.GetComponent<Rigidbody>();
                //myPoint.GetComponent<GrabbableSpringHelper>().springJoint.GetComponent<SpringJoint>().spring = 100000000000000f;
            }
            else if(myJoint.tag == "EndEffector")
            {
                myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<FixedJoint>().connectedBody = myJoint.GetComponent<Rigidbody>();
                myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<Rigidbody>().mass = 10f;
            }
            else
            {
                //Connect fixed joint to spring, and spring to hinge joint
                myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<FixedJoint>().connectedBody = myPoint.GetComponent<GrabbableSpringHelper>().springJoint.GetComponent<Rigidbody>();
                myPoint.GetComponent<GrabbableSpringHelper>().springJoint.GetComponent<SpringJoint>().connectedBody = myJoint.GetComponent<Rigidbody>();
                myPoint.GetComponent<GrabbableSpringHelper>().springJoint.GetComponent<SpringJoint>().spring = 100000f;
            }   
        }
    }

    public void AdjustGrabbable(Grabbable myGrabbable)
    {
        if(myJointID == 1)
        {
            myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>().InjectOptionalPositionConstraints(joint1PositionConstraints);
            myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>().InjectOptionalRotationConstraints(joint1RotationConstraints);
        }
        else if(myJointID == 2)
        {
            myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>().InjectOptionalPositionConstraints(joint2PositionConstraints);
            myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>().InjectOptionalRotationConstraints(joint2RotationConstraints);
        }

        myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>().Initialize(myGrabbable);

        myGrabbable.InjectOptionalTargetTransform(myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<Transform>());
        myGrabbable.InjectOptionalRigidbody(myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<Rigidbody>());
        myGrabbable.InjectOptionalOneGrabTransformer(myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>());
        myGrabbable.InjectOptionalTwoGrabTransformer(myPoint.GetComponent<GrabbableSpringHelper>().fixedJoint.GetComponent<GrabFreeTransformer>());
    }

    public void KillGrab(Grabbable myGrabbable)
    {
        //Reset Grabbable
        myGrabbable.InjectOptionalRigidbody(myJoint.GetComponent<Rigidbody>());
        myGrabbable.InjectOptionalTargetTransform(myJoint.GetComponent<Transform>());
        myGrabbable.InjectOptionalOneGrabTransformer(myJoint.GetComponent<GrabFreeTransformer>());
        myGrabbable.InjectOptionalTwoGrabTransformer(myJoint.GetComponent<GrabFreeTransformer>());

        Destroy(myPoint);
    }
}
