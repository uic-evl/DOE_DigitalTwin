using UnityEngine;

public class GetJointInstance : MonoBehaviour
{
    public GameObject pointController;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Awake()
    {
        //pointController = GameObject.Find("firefighter_diff_axis");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void KillGrabCall()
    {
        //pointController.GetComponent<CreateGrabPoint>().KillGrab();
    }
}
