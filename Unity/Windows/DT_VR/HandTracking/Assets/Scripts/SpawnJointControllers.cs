using UnityEngine;

public class SpawnJointControllers : MonoBehaviour
{

    public GameObject[] joints;
    public GameObject[] myControllers = new GameObject[7];

    public GameObject controllerPrefab;

    public bool controllersOn = true;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        /*
        for(int i = 0; i < joints.Length; i++)
        {
            myControllers[i] = Instantiate(controllerPrefab, joints[i].transform.position, joints[i].transform.rotation);
            myControllers[i].GetComponent<FixedJoint>().connectedBody = joints[i].GetComponent<Rigidbody>();
        }

        //DisableControllers();
        */
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void ToggleControllers()
    {
        if(controllersOn)
        {
            DisableControllers();
        }
        else
        {
            EnableControllers();
        }
    }

    public void DisableControllers()
    {
        if(controllersOn)
        {
            
            for(int i = 0; i < joints.Length; i++)
            {
                //myControllers[i].SetActive(false);
                joints[i].GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezePositionX | RigidbodyConstraints.FreezeRotationX |RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezePositionZ | RigidbodyConstraints.FreezeRotationZ;
            }

            controllersOn = false;
        }
    }

    public void EnableControllers()
    {
        if(!controllersOn)
        {
            for(int i = 0; i < joints.Length; i++)
            {
                joints[i].GetComponent<Rigidbody>().constraints = RigidbodyConstraints.None;
                /*
                myControllers[i].SetActive(true);
                myControllers[i].transform.position = joints[i].transform.position;
                myControllers[i].transform.rotation = joints[i].transform.rotation;*/
            }
        }

        controllersOn = true;
    }

}
