using System.Collections.Generic;
using UnityEngine;

abstract public class State
{
    public Vector3 pos;
    private float _cost;
    public State parent;
    public List<State> children;

    public State(Vector3 pos)
    {
        this.pos = pos;
        this._cost = -1;
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