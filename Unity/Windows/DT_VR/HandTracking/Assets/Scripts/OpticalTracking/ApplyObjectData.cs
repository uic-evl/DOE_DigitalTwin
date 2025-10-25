using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ApplyObjectData : MonoBehaviour
{
    public ObjectDataHandler dataHandler;
    public int myID;
    public int index;

    public int offsetID;

    public Vector3 offset;
    public Vector3 worldOffset;

    // Start is called before the first frame update
    void Awake()
    {
        for(int i = 0; i < dataHandler.numObjects; i++)
        {
            if(dataHandler.idLookup[i] == offsetID)
            {
                index = i;
            }
        }

        offset = new Vector3(dataHandler.objectList[index].posx * -1.0f, 0.745f, dataHandler.objectList[index].posz);
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

        Vector3 trackedPos = new Vector3(dataHandler.objectList[index].posx * -1.0f, dataHandler.objectList[index].posy, dataHandler.objectList[index].posz);
        gameObject.transform.position = trackedPos - offset + worldOffset;

        //gameObject.transform.rotation = Quaternion.Euler(new Vector3(dataHandler.objectList[index].orx, dataHandler.objectList[index].ory, dataHandler.objectList[index].orz));
        gameObject.transform.rotation = new Quaternion(dataHandler.objectList[index].orx, dataHandler.objectList[index].ory, dataHandler.objectList[index].orz, dataHandler.objectList[index].orw);

    }
}
