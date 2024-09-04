using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKManager : MonoBehaviour
{
    public RobotJoint[] Joints;
    public float[] angles;

    public float LearningRate;
    public float SamplingDistance;
    public float DistanceThreshold;

    public Transform target_xform;
    public Vector3 target;

    public Vector3 xaxis;
    public Vector3 yaxis;
    public Vector3 zaxis;

    void Start()
    {
        for(int i = 0; i < Joints.Length; i++)
        {
            angles[i] = getAngle(i, Joints[i]);

            //Debug.Log("Setting angle " + i.ToString() + " to " + getAngle(i, Joints[i]).ToString());
        }

    }

    void Update()
    {
        target = target_xform.localPosition;

        InverseKinematics(target, angles);

        for(int i = 0; i < Joints.Length; i++)
        {
            Joints[i].gameObject.transform.localRotation = Quaternion.AngleAxis(angles[i], Joints[i].Axis);
        }

        //Debug.Log(ForwardKinematics(angles));
        Debug.Log(DistanceFromTarget(target, angles));
        //Debug.Log(angles);

        //Debug.Log(Joints[6].gameObject.transform.localPosition);
    }

    public float getAngle(int i, RobotJoint joint)
    {
        //Debug.Log("Joint " + i.ToString() + " has axis" + joint.Axis.ToString());

        //X axis
        if(joint.Axis == xaxis)
        {
            return joint.gameObject.transform.localRotation.eulerAngles.x;
        }
        //Y axis
        else if(joint.Axis == yaxis)
        {
           return joint.gameObject.transform.localRotation.eulerAngles.y; 
        }
        //Z axis
        else if(joint.Axis == zaxis)
        {
           return joint.gameObject.transform.localRotation.eulerAngles.z; 
        }
        else
        {
            return 0;
        }

    }

    public Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;

        for(int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;
            
            prevPoint = nextPoint;
        }

        return prevPoint;
    }

    public float DistanceFromTarget(Vector3 target, float [] angles)
    {
        Vector3 point = ForwardKinematics (angles);
        return Vector3.Distance(point, target);
    }

    public float PartialGradient (Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];

        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = DistanceFromTarget(target, angles);

        angles[i] += SamplingDistance;
        float f_x_plus_d = DistanceFromTarget(target, angles);

        float gradient = (f_x_plus_d - f_x) / SamplingDistance;

        // Restores
        angles[i] = angle;

        return gradient;
    }

    public void InverseKinematics (Vector3 target, float []angles)
    {
        if (DistanceFromTarget(target, angles) < DistanceThreshold)
            return;

        for (int i = Joints.Length-1; i > 0; i--)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= LearningRate * gradient;

            // Clamp
            angles[i] = Mathf.Clamp(angles[i], Joints[i].MinAngle, Joints[i].MaxAngle);

            // Early termination
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
                return;
        }
    }
}
