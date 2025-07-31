using UnityEngine;
using UnityEditor.Animations;

public class RecordArmAnimation : MonoBehaviour
{
    public AnimationClip clip;

    private GameObjectRecorder m_Recorder;

    public bool isRecording = false;

    void Start()
    {
        m_Recorder = new GameObjectRecorder(gameObject);

        // Bind all ArticulationBodies of child joints
        m_Recorder.BindComponentsOfType<ArticulationBody>(gameObject, true);
    }

    void LateUpdate()
    {
        if (clip == null)
            return;

        if(isRecording)
        {
            // Take a snapshot and record all the bindings values for this frame.
            m_Recorder.TakeSnapshot(Time.deltaTime);
        }
    }

    public void onStopRecording()
    {
        if (clip == null)
            return;

        if (m_Recorder.isRecording)
        {
            // Save the recorded session to the clip.
            m_Recorder.SaveToClip(clip);
        }
    }

    public void toggleRecording()
    {
        if(!isRecording)
        {
            isRecording = true;
        }
        else if(isRecording)
        {
            isRecording = false;
        }
    }

    /*
    void OnDisable()
    {
        if (clip == null)
            return;

        if (m_Recorder.isRecording)
        {
            // Save the recorded session to the clip.
            m_Recorder.SaveToClip(clip);
        }
    }*/
}
