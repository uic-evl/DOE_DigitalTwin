using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class FetchData : MonoBehaviour
{
    public string json;

    void Start()
    {
        StartCoroutine(GetText());
    }

    void Update()
    {
        StartCoroutine(GetText());
    }

    IEnumerator GetText()
    {
        using (UnityWebRequest www = UnityWebRequest.Get("http://10.0.0.142:4000/mocap"))
        {
            yield return www.Send();

            if (www.isNetworkError || www.isHttpError)
            {
                Debug.Log(www.error);
            }
            else
            {
                // Show results as text
                json = www.downloadHandler.text;

                // Or retrieve results as binary data
                //byte[] results = www.downloadHandler.data;
            }
        }
    }
}
