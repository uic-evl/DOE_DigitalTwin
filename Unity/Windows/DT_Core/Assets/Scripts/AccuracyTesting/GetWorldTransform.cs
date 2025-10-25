using UnityEngine;

public class GetWorldTransform : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(gameObject.transform.position.ToString("F10"));
    }
}
