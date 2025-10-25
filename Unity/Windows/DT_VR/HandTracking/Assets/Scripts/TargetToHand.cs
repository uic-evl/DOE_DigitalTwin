using UnityEngine;

public class TargetToHand : MonoBehaviour
{
    public GameObject rightHand;
    public Vector3 offset;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.position = rightHand.transform.position + offset;
        gameObject.transform.rotation = rightHand.transform.rotation;
    }
}
