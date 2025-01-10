using UnityEngine;

public class GripperController : MonoBehaviour
{
    //Assume openGripper is active, and closedGripper is not at the start of the scene. 
    public GameObject openGripper;
    public GameObject closedGripper;

    public bool isOpen = true;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OpenGripper()
    {
        if(!isOpen)
        {
            closedGripper.SetActive(false);
            openGripper.SetActive(true);

            isOpen = true;
        }
    }

    public void CloseGripper()
    {
        if(isOpen)
        {
            openGripper.SetActive(false);
            closedGripper.SetActive(true);

            isOpen = false;
        }
    }

    public int GetGripperState()
    {
        if(isOpen)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
}
