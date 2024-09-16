using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ApplyObjectData : MonoBehaviour
{
    public ObjectDataHandler dataHandler;
    public int myID;
    public int index;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        for(int i = 0; i < dataHandler.numObjects; i++)
        {
            if(dataHandler.idLookup[i] == myID)
            {
                index = i;
            }
        }

        gameObject.transform.position = new Vector3(dataHandler.objectList[index].posx, dataHandler.objectList[index].posy, dataHandler.objectList[index].posz);
    }
}
