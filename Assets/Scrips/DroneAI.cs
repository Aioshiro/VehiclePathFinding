using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    //Path parameters
    private DroneController m_Drone; // the car controller we want to use
    [SerializeField] private Stack<StateDrone> my_path;

    //Maze parameters
    public TerrainManager terrain_manager;
    private Vector3 start_pos;
    private Vector3 goal_pos;
    private float terrainSize;
    private Vector2 terrainCenter;

    //RRT parameters
    [Header("RRT Settings")]
    private float fixedDeltaTime;
    [Tooltip("Radius where we consider we're close enough to the goal")]
    private float goalRadiusSquared;
    [SerializeField] private int numberOfIterations = 500;
    private float droneRadius;

    public GameObject currentGoalPos;
    private float maxAccel=15.0f;
    [SerializeField] private StateDrone currentState;
    [SerializeField] private float timeStepScaling = 1.0f;

    private void Start()
    {

        InitializeRRTParameters();

        // get the car controller
        m_Drone = GetComponent<DroneController>();
        maxAccel = m_Drone.max_acceleration;


        StateDrone initialState = new StateDrone(start_pos, 0, 0,(0,0));
        StateDrone finalState = RRT(initialState);
        ShowTree(initialState, Color.red);
        //ShowTree(rootGoal, Color.blue);
        ShowingTreeAndPath(finalState);
        currentState = my_path.Pop();
    }

    private void ShowingTreeAndPath(StateDrone finalStates)
    {
        //Recreating the path
        StateDrone currentState = finalStates;
        while (!(currentState.parent is null))
        {
            my_path.Push((StateDrone)currentState.parent);
            Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
            currentState = (StateDrone)currentState.parent;
        }
    }

    private void ShowTree(StateDrone currentState, Color color)
    {
        if (currentState.children.Count != 0)
        {
            foreach (StateDrone child in currentState.children)
            {
                Debug.DrawLine(currentState.pos, child.pos, color, Mathf.Infinity, false);
                ShowTree(child, color);
            }
        }
    }

    private void InitializeRRTParameters()
    {
        //Relative to Terrain
        start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;
        terrainCenter = new Vector2(terrain_manager.myInfo.x_high + terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high + terrain_manager.myInfo.z_low);
        terrainCenter /= 2;
        terrainSize = Mathf.Max(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / 2;
        goalRadiusSquared = FindObjectOfType<GameManager>().goal_tolerance/2;
        goalRadiusSquared *= goalRadiusSquared;
        droneRadius = this.GetComponent<SphereCollider>().radius+1f;


        //Relative to RRT
        StateDrone.InitializeInputs();
        fixedDeltaTime = Time.fixedDeltaTime*timeStepScaling;

        my_path = new Stack<StateDrone>();

    }

    private void FixedUpdate()
    {
        if (my_path.Count !=0)
        {
            MoveDrone();
        }
    }

    private void MoveDrone()
    {
        if (my_path.Count == 0)
        {
            return;
        }
        Vector3 projectedPos = new Vector3 (transform.position.x, 0, transform.position.z);
        if (Vector3.Distance(projectedPos, currentState.pos) < 2)
        {

            currentState = my_path.Pop();
            currentGoalPos.transform.position = currentState.pos;
        }
        float xAccel = (currentState.currentXSpeed - m_Drone.velocity.x) / (fixedDeltaTime * m_Drone.max_acceleration);
        float yAccel = (currentState.currentYSpeed - m_Drone.velocity.z) / (fixedDeltaTime * m_Drone.max_acceleration);
        m_Drone.Move(xAccel, yAccel);
    }

    private StateDrone RRT(StateDrone lastTreeState)
    {
        List<StateDrone> V = new List<StateDrone>
        {
            lastTreeState
        };
        for (int i = 0; i < numberOfIterations; i++)
        {
            Vector3 xRandom = SamplePointInMaze(goal_pos);
            StateDrone xNear = Nearest(V,xRandom);
            if (Control(xNear, xRandom, out StateDrone newState))
            {
                V.Add(newState);
                newState.parent = xNear;
                xNear.children.Add(newState);
                if ((newState.pos -goal_pos).sqrMagnitude < goalRadiusSquared)
                {
                    return newState;
                }
            }
        }
        return null;
    }


    private StateDrone NewState(StateDrone xNearest, float accelX, float accelY, float timeStep)
    {
        float newXSpeed = xNearest.currentXSpeed + accelX * maxAccel * timeStep;
        float newYSpeed = xNearest.currentYSpeed + accelY * maxAccel * timeStep;
        float totalVelocity = Mathf.Sqrt(newXSpeed * newXSpeed + newYSpeed * newYSpeed);
        if (totalVelocity > m_Drone.max_speed)
        {
            newXSpeed *= (m_Drone.max_speed / totalVelocity);
            newYSpeed *= (m_Drone.max_speed / totalVelocity);
        }
        Vector3 newPos = new Vector3(xNearest.pos.x + newXSpeed * timeStep, 0, xNearest.pos.z + newYSpeed * timeStep);
        return new StateDrone(newPos, newXSpeed, newYSpeed,(accelX,accelY));
    }

    private bool CollisionFree(Vector3 xStart, Vector3 xEnd)
    {
        //return (!Physics.Raycast(xStart, (xEnd - xStart).normalized, Vector3.Distance(xStart, xEnd), LayerMask.GetMask("Wall"))); //When the car is considered a point
        //return !Physics.CheckBox((xStart + xEnd) / 2, new Vector3(droneHalfWidth, 1, (xEnd - xStart).magnitude / 2 + droneHalfLength), Quaternion.LookRotation((xEnd - xStart).normalized, Vector3.up), LayerMask.GetMask("Wall")); // We consider the car as a box
        return (!Physics.SphereCast(xStart, droneRadius, xEnd - xStart,out _, (xEnd-xStart).magnitude, LayerMask.GetMask("Wall")));
    }

    private Vector3 SamplePointInMaze(Vector3 xGoal)
    {
        if (Random.value <= 0.9)
        {
            Vector3 xRand = new Vector3(2 * UnityEngine.Random.value - 1, 0, 2 * UnityEngine.Random.value - 1);
            xRand *= terrainSize; //Sampling random pos in the maze
            xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
            if (Physics.CheckSphere(xRand,1, LayerMask.GetMask("Wall")))
            {
                xRand = new Vector3(2 * UnityEngine.Random.value - 1, 0, 2 * UnityEngine.Random.value - 1);
                xRand *= terrainSize; //Sampling random pos in the maze
                xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
            }
            return xRand;
        }
        return xGoal; // sampling the goal 1% of the time to help convergence
    }

    private StateDrone Nearest(List<StateDrone> V, Vector3 xRand)
    {
        StateDrone xMin = null;
        StateDrone xMinPrime = null;
        float minDist = Mathf.Infinity;
        float minDistPrime = Mathf.Infinity;
        foreach (StateDrone x in V)
        {
            if (x.statesLeft.Count != 0)
            {
                float temp = (x.pos- xRand).sqrMagnitude;
                if (temp < minDistPrime)
                {
                    minDistPrime = temp;
                    xMinPrime = x;
                }
                if (Random.value> x.violationFrequency && temp< minDist)
                {
                    minDist = temp;
                    xMin = x;
                }
            }
        }
       if (minDist != Mathf.Infinity)
        {
            return xMin;
        }
        return xMinPrime;
    }


    private bool Control(StateDrone xNear, Vector3 xRand, out StateDrone newState)
    {
        float dMin = Mathf.Infinity;
        bool success = false;
        int uBest = -1;
        Stack<int> statesToRemove = new Stack<int>();
        newState = null;
        foreach (int i in xNear.statesLeft)
        {
            StateDrone xPrime = NewState(xNear, StateDrone.possibleInputs[i].Item1, StateDrone.possibleInputs[i].Item2, fixedDeltaTime);
            if (CollisionFree(xNear.pos, xPrime.pos))
            {
                float d = (xPrime.pos - xRand).sqrMagnitude;
                if (d < dMin)
                {
                    dMin = d;
                    success = true;
                    uBest = i;
                    newState = xPrime;
                }
            }
            else
            {
                statesToRemove.Push(i);
                UpdateTreeInfo(xNear);
            }
        }
        statesToRemove.Push(uBest);
        foreach (int i in statesToRemove)
        {
            xNear.statesLeft.Remove(i);
        }
        return success;
    }


    private void UpdateTreeInfo(StateDrone xNear)
    {
        float m = StateDrone.numberOfPossibleInputs;
        float p = 1 / m;
        xNear.violationFrequency += p;
        p *= p;
        StateDrone x1 = xNear;
        while (!(x1.parent is null))
        {
            x1 = (StateDrone) x1.parent;
            x1.violationFrequency += p;
            p /= m;
        }
    }

}

