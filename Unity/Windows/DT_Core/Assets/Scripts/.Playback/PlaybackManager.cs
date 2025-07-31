using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PlaybackManager : MonoBehaviour
{
    public List<JointData> jointFrames;

    public bool isRecording;
    public bool isPlaying;

    public ArmControllerOmniverseZMQ controlArm;
    public ArmControllerPlayback playbackArm;

    private JointData thisJointData;

    public int frameNum;

    [SerializeField]
    private string basePath; 
    public string outfile;
    public StreamWriter writer;

    // Start is called before the first frame update
    void Start()
    {
        writer = new StreamWriter(basePath + outfile, true);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(isRecording)
        {
            thisJointData = new JointData();
            thisJointData.fillFromArray(controlArm.myAngles);
            jointFrames.Add(thisJointData);
        }
        else if(isPlaying)
        {
            if(frameNum < jointFrames.Count)
            {
                playbackArm.UpdateJoints(jointFrames[frameNum]);
                frameNum++;
            }
            else if(frameNum >= jointFrames.Count)
            {
                frameNum = 0;
            }
        }
    }

    public void startRecord()
    {
        isRecording = true;
    }

    public void stopRecord()
    {
        isRecording = false;
    }

    public void startPlay()
    {
        isPlaying = true;
        frameNum = 0;
    }

    public void stopPlay()
    {
        isPlaying = false;
    }

    public void saveRecording()
    {
        if(isPlaying == false && isRecording == false)
        {
            for(int i = 0; i < jointFrames.Count; i++)
            {
                writer.WriteLine(jointFrames[i].printString());
            }
        }
    }

    void OnDestroy()
    {
        writer.Close();
    }

}
