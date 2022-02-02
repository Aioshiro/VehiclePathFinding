using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    //Path parameters
    private DroneController m_Drone; // the car controller we want to use
    [SerializeField] private List<StateDrone> my_path;

    //Maze parameters
    public TerrainManager terrain_manager;
    private Vector3 start_pos;
    private Vector3 goal_pos;
    private float terrainSize;
    private Vector2 terrainCenter;

    //RRT parameters
    [Header("RRT Settings")]
    private float fixedDeltaTime;
    private bool arrivedAtGoal = false;
    [SerializeField] private float validationDistance = 1.0f;
    [Tooltip("Radius where we consider we're close enough to the goal")]
    private float goalRadius;
    [SerializeField] private int numberOfIterations = 500;
    private float droneRadius;

    private StateDrone currentGoal = new StateDrone(Vector3.zero, 0, 0);
    public GameObject currentGoalPos;
    private StateDrone rootStart;
    private StateDrone rootGoal;
    private KdTree kdTreeStart;
    private KdTree kdTreeGoal;
    private float maxAccel=15.0f;


    private void Start()
    {
        InitializeRRTParameters();

        // get the car controller
        m_Drone = GetComponent<DroneController>();
        maxAccel = m_Drone.max_acceleration;


        // Plot your path to see if it makes sense
        // Note that path can only be seen in "Scene" window, not "Game" window
        (StateDrone,StateDrone) meetingPoint = RRT(start_pos, goal_pos);

        ShowTree(rootStart, Color.red);
        ShowTree(rootGoal, Color.blue);
        ShowingTreeAndPath(meetingPoint);

    }

    private void ShowingTreeAndPath((StateDrone, StateDrone) finalStates)
    {
        //Recreating the path
        StateDrone currentState = finalStates.Item1;
        my_path.Add(currentState);
        while (!(currentState.parent is null))
        {
            my_path.Add((StateDrone)currentState.parent);
            Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
            currentState = (StateDrone)currentState.parent;
        }
        my_path.Reverse();
        currentState = finalStates.Item2;
        my_path.Add(currentState);
        while (!(currentState.parent is null))
        {
            my_path.Add((StateDrone)currentState.parent);
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
        goalRadius = FindObjectOfType<GameManager>().goal_tolerance;
        droneRadius = this.GetComponent<SphereCollider>().radius +0.1f;


        //Relative to RRT
        fixedDeltaTime = Time.fixedDeltaTime;
        rootStart = new StateDrone(start_pos, 0, 0);
        rootGoal = new StateDrone(goal_pos, 0, 0);
        //Finding the orientation of the goal
        kdTreeStart = new KdTree(true)
            {
                rootStart
            };
        kdTreeGoal = new KdTree(true)
            {
                rootGoal
            };

        my_path = new List<StateDrone>();

    }

    private void FixedUpdate()
    {
        if (!(my_path[0] is null))
        {
            MoveDrone();
            currentGoalPos.transform.position = my_path[0].pos;
        }
    }

    private void MoveDrone()
    {
        if (my_path.Count == 0)
        {
            arrivedAtGoal = true;
            return;
        }
        Vector3 projectedPosition = new Vector3(this.transform.position.x, 0, transform.position.z);
        while (Vector3.Distance(my_path[0].pos, projectedPosition) < validationDistance)
        {
            my_path.RemoveAt(0);
        }
        maxAccel = m_Drone.max_acceleration;
        currentGoal = my_path[0];
        currentGoalPos.transform.position = currentGoal.pos;
        (float, float) inputs = GoTo(currentGoal.pos);
        m_Drone.Move(inputs.Item1, inputs.Item2);
    }



    private (StateDrone,StateDrone) RRT(Vector3 xStart, Vector3 xGoal)
    {
        int i = 0;
        float minCost = Mathf.Infinity;
        (StateDrone, StateDrone) bestResult = (null,null);
        while (i < numberOfIterations)
        {
            i += 1;
            Vector3 xRand = SamplePointInMaze(true);
            StateDrone xNew = Extend(rootStart,xRand,true);
            if (!(xNew is null) && Connect(xNew, out StateDrone otherTreeState, false))
            {
                if (xNew.Cost() + otherTreeState.Cost() < minCost)
                {
                    minCost = xNew.Cost() + otherTreeState.Cost();
                    bestResult = (xNew, otherTreeState);
                }
            }
            StateDrone xOtherTree;
            if (!(xNew is null))
            {
                xOtherTree = Extend(rootGoal, xNew.pos, false);
                if (!(xOtherTree is null) && Connect(xOtherTree,out otherTreeState, true))
                {
                    if (otherTreeState.Cost() + xOtherTree.Cost() < minCost)
                    {
                        minCost = otherTreeState.Cost() + xOtherTree.Cost();
                        bestResult = (otherTreeState, xOtherTree);
                    }

                }
            }
            else
            {
                Extend(rootGoal, xRand, false);
            }
        }
        Debug.Log(minCost);
        return bestResult;
    }

    private StateDrone Extend(StateDrone treeRoot,Vector3 xRand, bool startTree)
    {
        //Sampling random pos in the maze
        //Finding nearest point of xRand in the graph
        StateDrone xNearest = Nearest(xRand,startTree);
        StateDrone newState = Steer(xNearest, xRand);
        if (CollisionFree(xNearest.pos, newState.pos)) // If we can go to xNearest to xNew, we add xNew to the graph
        {
            if (startTree)
            {
                kdTreeStart.Add(newState);
            }
            else
            {
                kdTreeGoal.Add(newState);
            }
            newState.parent = xNearest;
            xNearest.children.Add(newState);
            return newState;
        }
        return null;
    }

    private bool Connect(StateDrone xNew, out StateDrone otherTreeState, bool isStartTree)
    {
        otherTreeState = Nearest(xNew.pos, isStartTree);
        if ((otherTreeState.pos-xNew.pos).sqrMagnitude < validationDistance*validationDistance/10)
        {
            return true;
        }
        return false;
    }
    private StateDrone NewState(StateDrone xNearest, float accelX, float accelY, float timeStep)
    {
        (float, float, float, float) values = (xNearest.pos.x, xNearest.pos.z, xNearest.currentXSpeed, xNearest.currentYSpeed);
        (float, float) derivatives = GetDerivatives(values, timeStep, accelX, accelY);
        float newXSpeed = values.Item3 + derivatives.Item1 * timeStep;
        newXSpeed = Mathf.Clamp(newXSpeed, -m_Drone.max_speed, m_Drone.max_speed);
        float newYSpeed = values.Item4 + derivatives.Item2 * timeStep;
        newYSpeed = Mathf.Clamp(newYSpeed, -m_Drone.max_speed, m_Drone.max_speed);
        Vector3 newPos = new Vector3(values.Item1 + newXSpeed * timeStep, 0, values.Item2 + newYSpeed * timeStep);
        return new StateDrone(newPos, newXSpeed, newYSpeed);
    }

    private (float, float) GetDerivatives((float, float, float, float) state, float timeStep, float accelX, float accelY)
    {
        return (accelX * maxAccel, accelY * maxAccel); 
    }

    private bool CollisionFree(Vector3 xStart, Vector3 xEnd)
    {
        //return (!Physics.Raycast(xStart, (xEnd - xStart).normalized, Vector3.Distance(xStart, xEnd), LayerMask.GetMask("Wall"))); //When the car is considered a point
        //return !Physics.CheckBox((xStart + xEnd) / 2, new Vector3(droneHalfWidth, 1, (xEnd - xStart).magnitude / 2 + droneHalfLength), Quaternion.LookRotation((xEnd - xStart).normalized, Vector3.up), LayerMask.GetMask("Wall")); // We consider the car as a box
        return (!Physics.SphereCast(xStart, droneRadius, xEnd - xStart,out _, (xEnd-xStart).magnitude, LayerMask.GetMask("Wall")));
    }

    private Vector3 SamplePointInMaze(bool isStartTree)
    {
        if (Random.value <= 0.99)
        {
            Vector3 xRand = new Vector3(2 * UnityEngine.Random.value - 1, 0, 2 * UnityEngine.Random.value - 1);
            xRand *= terrainSize; //Sampling random pos in the maze
            xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
            return xRand;
        }
        if (isStartTree)
        {
            return goal_pos; // sampling the goal 1% of the time to help convergence
        }
        return start_pos;
    }

    private StateDrone Nearest(Vector3 xRand, bool isStartTree)
    {
        //State xMin = new State(Vector3.zero, 0, 0, 0, 0);
        //float minDist = Mathf.Infinity;
        //foreach (State x in V)
        //{
        //    float temp = Vector3.Distance(x.pos, xRand);
        //    if (temp < minDist)
        //    {
        //        minDist = temp;
        //        xMin = x;
        //    }
        //}
        //return xMin;
        if (isStartTree)
        {
            return (StateDrone)kdTreeStart.FindClosest(xRand);
        }
        return (StateDrone)kdTreeGoal.FindClosest(xRand);
    }

    private StateDrone Steer(StateDrone xNearest, Vector3 xRand)
    {
        float distance = Mathf.Infinity;
        StateDrone finalState = new StateDrone(Vector3.zero, 0, 0);
        for (float accelX = -1; accelX < 1.01; accelX += 0.05f)
        {
            for (float accelY = -1; accelY < 1.01; accelY += 0.05f)
            {
                if (accelX * accelX + accelY * accelY < 1)
                {
                    StateDrone newState = NewState(xNearest, accelX, accelY, fixedDeltaTime);
                    float newDistance = (newState.pos - xRand).sqrMagnitude;
                    if (newDistance < distance)
                    {
                        distance = newDistance;
                        finalState = newState;
                    }
                }
            }
        }
        return finalState;
    }

    private (float,float) GoTo(Vector3 xGoal)
    {
        float distance = Mathf.Infinity;
        Vector3 projectedPosition = new Vector3(transform.position.x, 0, transform.position.z);
        StateDrone CurrentState = new StateDrone(projectedPosition, m_Drone.velocity.x, m_Drone.velocity.z);
        float finalAccelX = 0;
        float finalAccelY = 0;
        for (float accelX = -1; accelX < 1.01; accelX += 0.05f)
        {
            for (float accelY = -1; accelY < 1.01; accelY += 0.05f)
            {
                if (accelX * accelX + accelY * accelY < 1)
                {
                    StateDrone newState = NewState(CurrentState, accelX, accelY, fixedDeltaTime);
                    float newDistance = Vector3.Distance(newState.pos, xGoal);
                    if (newDistance < distance)
                    {
                        distance = newDistance;
                        finalAccelX = accelX;
                        finalAccelY = accelY;
                    }
                }
            }
        }
        return (finalAccelX, finalAccelY);
    }
}

