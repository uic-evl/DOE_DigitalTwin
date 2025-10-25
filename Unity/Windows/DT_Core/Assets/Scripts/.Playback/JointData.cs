using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class JointData
{
    public float joint1;
    public float joint2;
    public float joint3;
    public float joint4;
    public float joint5;
    public float joint6;
    public float joint7;

    public JointData()
    {
        joint1 = 0f;
        joint2 = 0f;
        joint3 = 0f;
        joint4 = 0f;
        joint5 = 0f;
        joint6 = 0f;
        joint7 = 0f;
    }

    public void fillFromArray(float[] input)
    {
        joint1 = input[0];
        joint2 = input[1];
        joint3 = input[2];
        joint4 = input[3];
        joint5 = input[4];
        joint6 = input[5];
        joint7 = input[6];
    }

    public string printString()
    {
        string outString = joint1.ToString("F5") + "," + joint2.ToString("F5") + "," + joint3.ToString("F5") + "," +
        joint4.ToString("F5") + "," + joint5.ToString("F5") + "," + joint6.ToString("F5") + "," + joint7.ToString("F5");

        return outString;
    }
}
