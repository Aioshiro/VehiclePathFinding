using System.Collections.Generic;
using UnityEngine;

public class StateCar : State
{
    public float currentSpeed;
    public float currentAngle;
    public static (float, float)[] possibleInputs;
    public static int numberOfPossibleInputs;
    public List<int> statesLeft;
    public float violationFrequency;


    public StateCar(Vector3 pos, float currentSpeed, float currentAngle): base(pos)
    {
        this.currentSpeed = currentSpeed;
        this.currentAngle = currentAngle;
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

    public static void InitializeInputs()
    {
        possibleInputs = new (float, float)[]
        {
            ///TO FILL
        };

        numberOfPossibleInputs = possibleInputs.Length;

    }


}