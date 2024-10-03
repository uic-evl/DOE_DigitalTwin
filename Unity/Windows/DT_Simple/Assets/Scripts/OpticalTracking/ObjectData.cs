using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class ObjectData
{
    public int sourceID;

    public float posx;
    public float posy;
    public float posz;

    public float orw;
    public float orx;
    public float ory;
    public float orz;

    public ObjectData()
    {
        sourceID = 0;

        posx = 0f;
        posy = 0f;
        posz = 0f;

        orw = 0f;
        orx = 0f;
        ory = 0f;
        orz = 0f;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
