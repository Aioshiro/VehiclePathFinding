using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StateDrone : State
{
    public float currentXSpeed;
    public float currentYSpeed;

    public StateDrone(Vector3 pos, float currentXSpeed, float currentYSpeed) : base(pos)
    {
        this.currentXSpeed = currentXSpeed;
        this.currentYSpeed = currentYSpeed;
    }
}
