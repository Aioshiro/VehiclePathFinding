using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class StateDrone : State
{
    public float currentXSpeed;
    public float currentYSpeed;
    public static (float, float)[] possibleInputs;
    public static int numberOfPossibleInputs;
    public List<int> statesLeft;
    public float violationFrequency;
    public (float, float) inputFromLastState;

    public static void InitializeInputs()
    {
        float maxAccel = 0.6f;
        possibleInputs = new (float,float)[9]
        {
            (0, 0),
            (maxAccel, 0),
            (-maxAccel,0),
            (0,maxAccel),
            (0,-maxAccel),
            (maxAccel,maxAccel),
            (-maxAccel,-maxAccel),
            (maxAccel,-maxAccel),
            (-maxAccel,maxAccel)
        };
        //for (float i = -1; i < 1.01f; i += 0.25f)
        //{
        //    for (float j = -1; j < 1.01f; j += 0.25f)
        //    {
        //        if (i * i + j * j < 1)
        //        {
        //            possibleInputs.Add((i, j));
        //        }
        //    }
        //}


        numberOfPossibleInputs = possibleInputs.Length;

    }

    public StateDrone(Vector3 pos, float currentXSpeed, float currentYSpeed,(float,float) input) : base(pos)
    {
        this.currentXSpeed = currentXSpeed;
        this.currentYSpeed = currentYSpeed;
        this.inputFromLastState = input;
        violationFrequency = 0;
        statesLeft = new List<int>
        {
            Capacity = numberOfPossibleInputs
        };
        for (int i = 0; i < numberOfPossibleInputs; i++)
        {
            statesLeft.Add(i);
        }
    }
}