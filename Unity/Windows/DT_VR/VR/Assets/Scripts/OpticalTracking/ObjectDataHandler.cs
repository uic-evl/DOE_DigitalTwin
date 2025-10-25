using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectDataHandler : MonoBehaviour
{
    public FetchData fetchData;

    //This should match the number of objects being sent from the oinputServer. 
    public int numObjects;
    public int objectOffset = 18;

    public int[] idLookup;

    public List<ObjectData> objectList;

    public List<GameObject> trackedObjects;
    
    void Start()
    {
        idLookup = new int[numObjects];

        objectList = new List<ObjectData>();

        for(int i = 0; i < numObjects; i++)
        {
            objectList.Add(new ObjectData());
        }
    }

    // Update is called once per frame
    void Update()
    {
        string[] data = fetchData.json.Split(',');

        if(data.Length > 1)
        {
            for(int i = 0; i < numObjects; i++)
            {
                int myOffset = objectOffset * i;

                objectList[i].sourceID = int.Parse(data[2 + myOffset].Split(':')[1]); 
                idLookup[i] = objectList[i].sourceID;

                objectList[i].posx = float.Parse(data[7 + myOffset].Split(':')[1]);
                objectList[i].posy = float.Parse(data[8 + myOffset].Split(':')[1]);
                objectList[i].posz = float.Parse(data[9 + myOffset].Split(':')[1]);

                objectList[i].orw = float.Parse(data[10 + myOffset].Split(':')[1]);
                objectList[i].orx = float.Parse(data[11 + myOffset].Split(':')[1]);
                objectList[i].ory = float.Parse(data[12 + myOffset].Split(':')[1]);
                objectList[i].orz = float.Parse(data[13 + myOffset].Split(':')[1]);
            }
            
            if(!trackedObjects[0].activeSelf)
            {
                for(int i = 0; i < trackedObjects.Count; i++)
                {
                    trackedObjects[i].SetActive(true);
                }
            }
        }
    }
}
