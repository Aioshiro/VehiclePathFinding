using System.Collections.Generic;
using UnityEngine;

public class StateCar : State
{
    public float currentSpeed;
    public float currentAngle;

    public StateCar(Vector3 pos, float currentSpeed, float currentAngle): base(pos)
    {
        this.currentSpeed = currentSpeed;
        this.currentAngle = currentAngle;
    }

}