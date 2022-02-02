using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StateDrone : State
{
    public float currentXSpeed;
    public float currentYSpeed;
    public float accelXFromLastState;
    public float accelYFromLastState;

    public StateDrone(Vector3 pos, float currentXSpeed, float currentYSpeed, float accelXFromLastState, float accelYFromLastState) : base(pos)
    {
        this.currentXSpeed = currentXSpeed;
        this.currentYSpeed = currentYSpeed;
        this.accelXFromLastState = accelXFromLastState;
        this.accelYFromLastState = accelYFromLastState;
    }
}
