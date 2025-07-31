using UnityEngine;

public class PlayRecordedClip : MonoBehaviour
{
    public Animator animator;

    void Start()
    {
        animator.enabled = false;
    }

    public void onPlay()
    {
        animator.enabled = true;
    }
}
