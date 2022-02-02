using System.Collections.Generic;
using UnityEngine;

public class StateCar : State
{
    private float _cost;
    public float currentSpeed;
    public float currentAngle;
    public float accelFromLastState;
    public float steeringFromLastState;

    public StateCar(Vector3 pos, float currentSpeed, float currentAngle, float accelFromLastState, float steeringFromLastState): base(pos)
    {
        this.currentSpeed = currentSpeed;
        this.currentAngle = currentAngle;
        this.accelFromLastState = accelFromLastState;
        this.steeringFromLastState = steeringFromLastState;
    }

}