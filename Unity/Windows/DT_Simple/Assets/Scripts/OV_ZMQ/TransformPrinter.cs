using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TransformPrinter : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(gameObject.transform.rotation.eulerAngles.ToString("F5"));
    }
}
