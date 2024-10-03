using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class position : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log(gameObject.transform.position); 
        Debug.Log(gameObject.transform.localPosition); 
    }

    // Update is called once per frame
    void Update()
    {

    }
}
