using UnityEngine;

public class EnableGrabbablePoint : MonoBehaviour
{
    public GameObject grabbablePoint;
    public GameObject selectedJoint;
    private Vector3 offset = new Vector3(0.0f, 0.02f, 0.0f);


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(grabbablePoint.activeSelf)
        {
            grabbablePoint.transform.localPosition = selectedJoint.transform.localPosition + offset;
            grabbablePoint.transform.rotation = selectedJoint.transform.rotation;
        }
    
    }
    
    public void Enable()
    {
        grabbablePoint.SetActive(true);
    }

    public void Disable()
    {
        grabbablePoint.SetActive(false);
    }
}
