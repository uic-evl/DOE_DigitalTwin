using UnityEngine;
using UnityEngine.InputSystem;

public class MoveObjectFromController : MonoBehaviour
{
    public InputActionReference trackPoseActionLeft;
    public InputActionReference trackPoseActionRight;

    public GameObject leftController;
    public GameObject rightController;

    GameObject myController;
    InputActionReference myAction;

    public bool isTracking = false;
    public bool isRightHanded = true;

    Vector3 startPos;
    Vector3 startRot;

    Vector3 deltaPos;
    Vector3 deltaRot;

    Vector3 objectStartPos;
    Vector3 objectStartRot;

    void Start()
    {
        if(isRightHanded)
        {
            myController = rightController;
            myAction = trackPoseActionRight;
        }
        else
        {
            myController = leftController;
            myAction = trackPoseActionLeft;
        }
    }

    void Update()
    {
        if(myAction.action.WasPressedThisFrame())
        {
            if(!isTracking)
            {
                objectStartPos = gameObject.transform.position;
                objectStartRot = gameObject.transform.rotation.eulerAngles;

                startPos = myController.transform.position;
                startRot = myController.transform.rotation.eulerAngles;
            }
            isTracking = true;
        }

        if(isTracking)
        {
            deltaPos = myController.transform.position - startPos;
            deltaRot = myController.transform.rotation.eulerAngles - startRot;

            gameObject.transform.position = objectStartPos + deltaPos;
            gameObject.transform.eulerAngles = myController.transform.rotation.eulerAngles;
        }

        if(myAction.action.WasReleasedThisFrame())
        {
            isTracking = false;
        }
    }
}
