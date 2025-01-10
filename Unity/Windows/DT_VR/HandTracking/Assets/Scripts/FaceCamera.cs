using UnityEngine;

public class FaceCamera : MonoBehaviour
{
    public GameObject mainCameraRig;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.rotation = mainCameraRig.transform.rotation;
    }
}
