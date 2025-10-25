using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SnapToController : MonoBehaviour
{

    public Transform parentController;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.position = new Vector3(parentController.position.x, parentController.position.y, parentController.position.z);
        gameObject.transform.rotation = parentController.rotation;
    }
}
