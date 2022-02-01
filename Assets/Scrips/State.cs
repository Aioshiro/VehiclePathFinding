using System.Collections.Generic;
using UnityEngine;

public class State
{
    public Vector3 pos;
    private float _cost;
    public float currentSpeed;
    public float currentAngle;
    public float accelFromLastState;
    public float steeringFromLastState;
    public State parent;
    public List<State> children;

    public State(Vector3 pos, float currentSpeed, float currentAngle, float accelFromLastState, float steeringFromLastState)
    {
        this.pos = pos;
        this._cost = -1;
        this.currentSpeed = currentSpeed;
        this.currentAngle = currentAngle;
        this.accelFromLastState = accelFromLastState;
        this.steeringFromLastState = steeringFromLastState;
        this.parent = null;
        this.children = new List<State>();
    }

    public float Cost()
    {
        if (this._cost != -1)
        {
            return this._cost;
        }
        float cost = parent.Cost() + Vector3.Distance(this.pos, parent.pos);
        this._cost = cost;
        return cost;
    }
}