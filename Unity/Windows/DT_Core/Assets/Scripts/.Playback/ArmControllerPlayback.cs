using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmControllerPlayback : MonoBehaviour
{
    // Add the joints in the order they are recieved from Omniverse
    public ArticulationBody[] JointArticulationBodies;

    private int NumRobotJoints;

    [SerializeField]
    GameObject myArm;

    public PlaybackManager myPlayback;

    void Start()
    {
        Debug.Log("Initializing");
        NumRobotJoints = JointArticulationBodies.Length;
    }

    void Update()
    {

    }

    // Update the joint angle by setting the
    // xDrive.Target of each ArticulationBody
    public void UpdateJointAngle(float cmd, int joint)
    {
        float angle = cmd;

        var jointXDrive = JointArticulationBodies[joint].xDrive;
        jointXDrive.target = angle;
        JointArticulationBodies[joint].xDrive = jointXDrive;
    }

    public void UpdateJoints(JointData frame)
    {
        UpdateJointAngle(frame.joint1, 0);
        UpdateJointAngle(frame.joint2, 1);
        UpdateJointAngle(frame.joint3, 2);
        UpdateJointAngle(frame.joint4, 3);
        UpdateJointAngle(frame.joint5, 4);
        UpdateJointAngle(frame.joint6, 5);
        UpdateJointAngle(frame.joint7, 6);
    }
}
