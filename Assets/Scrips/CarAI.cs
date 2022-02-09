using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Random = UnityEngine.Random;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {

        //Path parameters
        private CarController m_Car; // the car controller we want to use
        [SerializeField] private Stack<StateCar> my_path;

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

        public GameObject currentGoalPos;
        [SerializeField] private StateCar currentState;
        [SerializeField] private float timeStepScaling = 1.0f;

        private void Start()
        {

            InitializeRRTParameters();

            // get the car controller
            m_Car = GetComponent<CarController>();


            StateCar initialState = new StateCar(start_pos, 0, 0);
            StateCar finalState = RRT(initialState);
            ShowTree(initialState, Color.red);
            ShowingTreeAndPath(finalState);
            currentState = my_path.Pop();
        }

        private void ShowingTreeAndPath(StateCar finalStates)
        {
            //Recreating the path
            StateCar currentState = finalStates;
            while (!(currentState.parent is null))
            {
                my_path.Push((StateCar)currentState.parent);
                Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
                currentState = (StateCar)currentState.parent;
            }
        }

        private void ShowTree(StateCar currentState, Color color)
        {
            //Drawing the tree
            if (currentState.children.Count != 0)
            {
                foreach (StateCar child in currentState.children)
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
            goalRadiusSquared = FindObjectOfType<GameManager>().goal_tolerance / 2;
            goalRadiusSquared *= goalRadiusSquared;


            //Relative to RRT
            StateCar.InitializeInputs();
            fixedDeltaTime = Time.fixedDeltaTime * timeStepScaling;

            my_path = new Stack<StateCar>();

        }

        private void FixedUpdate()
        {
            if (my_path.Count != 0)
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
            Vector3 projectedPos = new Vector3(transform.position.x, 0, transform.position.z);
            if (Vector3.Distance(projectedPos, currentState.pos) < 2)
            {

                currentState = my_path.Pop();
                currentGoalPos.transform.position = currentState.pos;
            }
            float steering = 0; //To compute
            float accel = 0; //To compute
            float handbrake = 0; //To compute
            m_Car.Move(steering, accel, accel, handbrake);
        }

        private StateCar RRT(StateCar lastTreeState)
        {
            List<StateCar> V = new List<StateCar>
        {
            lastTreeState
        };
            for (int i = 0; i < numberOfIterations; i++)
            {
                Vector3 xRandom = SamplePointInMaze(goal_pos);
                StateCar xNear = Nearest(V, xRandom);
                if (Control(xNear, xRandom, out StateCar newState))
                {
                    V.Add(newState);
                    newState.parent = xNear;
                    xNear.children.Add(newState);
                    if ((newState.pos - goal_pos).sqrMagnitude < goalRadiusSquared)
                    {
                        return newState;
                    }
                }
            }
            return null;
        }


        private StateCar NewState(StateCar xNearest, float accelX, float accelY, float timeStep)
        {
            Vector3 newPos = xNearest; //To compute
            float newSpeed = StateCar.currentSpeed + fixedTimeScale * (StateCar.currentSpeed*timeScale); //(eulers formula) v(t + fixedTimeScale) = v(t) + fixedTimeScale * acceleration(t)
            float newAngle = StateCar.currentSpeed/1 * Math.Tan(currentAngle); // need length or car
            return new StateCar(newPos, newSpeed, newAngle);
        }

        private bool CollisionFree(Vector3 xStart, Vector3 xEnd)
        {
            ///Maybe to change for the car 
            //return !Physics.CheckBox((xStart + xEnd) / 2, new Vector3(droneHalfWidth, 1, (xEnd - xStart).magnitude / 2 + droneHalfLength), Quaternion.LookRotation((xEnd - xStart).normalized, Vector3.up), LayerMask.GetMask("Wall")); // We consider the car as a box
            return (!Physics.SphereCast(xStart, 5, xEnd - xStart, out _, (xEnd - xStart).magnitude, LayerMask.GetMask("Wall")));
        }

        private Vector3 SamplePointInMaze(Vector3 xGoal)
        {
            if (Random.value <= 0.9)
            {
                Vector3 xRand = new Vector3(2 * Random.value - 1, 0, 2 * Random.value - 1);
                xRand *= terrainSize; //Sampling random pos in the maze
                xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
                if (Physics.CheckSphere(xRand, 1, LayerMask.GetMask("Wall")))
                {
                    xRand = new Vector3(2 * Random.value - 1, 0, 2 * Random.value - 1);
                    xRand *= terrainSize; //Sampling random pos in the maze
                    xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
                }
                return xRand;
            }
            return xGoal; // sampling the goal 1% of the time to help convergence
        }

        private StateCar Nearest(List<StateCar> V, Vector3 xRand)
        {
            StateCar xMin = null;
            StateCar xMinPrime = null;
            float minDist = Mathf.Infinity;
            float minDistPrime = Mathf.Infinity;
            foreach (StateCar x in V)
            {
                if (x.statesLeft.Count != 0)
                {
                    float temp = (x.pos - xRand).sqrMagnitude;
                    if (temp < minDistPrime)
                    {
                        minDistPrime = temp;
                        xMinPrime = x;
                    }
                    if (Random.value > x.violationFrequency && temp < minDist)
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


        private bool Control(StateCar xNear, Vector3 xRand, out StateCar newState)
        {
            float dMin = Mathf.Infinity;
            bool success = false;
            int uBest = -1;
            Stack<int> statesToRemove = new Stack<int>();
            newState = null;
            foreach (int i in xNear.statesLeft)
            {
                StateCar xPrime = NewState(xNear, StateCar.possibleInputs[i].Item1, StateCar.possibleInputs[i].Item2, fixedDeltaTime);
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


        private void UpdateTreeInfo(StateCar xNear)
        {
            float m = StateCar.numberOfPossibleInputs;
            float p = 1 / m;
            xNear.violationFrequency += p;
            p *= p;
            StateCar x1 = xNear;
            while (!(x1.parent is null))
            {
                x1 = (StateCar)x1.parent;
                x1.violationFrequency += p;
                p /= m;
            }
        }
    }
}
