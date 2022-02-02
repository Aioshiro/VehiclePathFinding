using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        //Path parameters
        private CarController m_Car; // the car controller we want to use
        [SerializeField] private List<StateCar> my_path;

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
        [SerializeField] private float carLength;
        [SerializeField] private float carHalfWidth;
        private StateCar currentGoal = new StateCar(Vector3.zero,0,0);
        public GameObject currentGoalPos;
        private StateCar rootStart;
        private StateCar rootGoal;
        private KdTree kdTreeStart;
        private KdTree kdTreeGoal;


        private void Start()
        {
            InitializeRRTParameters();

            // get the car controller
            m_Car = GetComponent<CarController>();


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            (StateCar,StateCar) meetingPoints = RRT(start_pos, goal_pos);

            ShowTree(rootStart,Color.red);
            ShowTree(rootGoal,Color.blue);
            ShowingTreeAndPath(meetingPoints);

        }

        private void ShowingTreeAndPath((StateCar,StateCar) finalStates)
        {
            //Recreating the path
            StateCar currentState = finalStates.Item1;
            my_path.Add(currentState);
            while (!(currentState.parent is null))
            {
                my_path.Add((StateCar) currentState.parent);
                Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
                currentState= (StateCar) currentState.parent;
            }
            my_path.Reverse();
            currentState = finalStates.Item2;
            my_path.Add(currentState);
            while (!(currentState.parent is null))
            {
                my_path.Add((StateCar) currentState.parent);
                Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
                currentState =(StateCar) currentState.parent;
            }
        }

        private void ShowTree(StateCar currentState,Color color)
        {
            if (currentState.children.Count != 0)
            {
                foreach( StateCar child in currentState.children)
                {
                    Debug.DrawLine(currentState.pos, child.pos, color, Mathf.Infinity, false);
                    ShowTree(child,color);
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

            //Relative to RRT
            fixedDeltaTime = Time.fixedDeltaTime;
            rootStart = new StateCar(start_pos, 0, Mathf.Deg2Rad * 90);

            //Finding the orientation of the goal
            float maxDistance = -Mathf.Infinity;
            float angle = 0;
            for (int i = 0; i< 4;i++)
            {
                if (Physics.Raycast(goal_pos,new Vector3(Mathf.Cos(Mathf.PI/2*i),0, Mathf.Sin(Mathf.PI / 2 * i)), out RaycastHit hit, LayerMask.GetMask("Wall")))
                {
                    if (hit.distance > maxDistance)
                    {
                        maxDistance = hit.distance;
                        angle = Mathf.PI / 2 * i;
                    }
                }
                else
                {
                    maxDistance = Mathf.Infinity;
                    angle = Mathf.PI / 2 * i;
                }
            }
            rootGoal = new StateCar(goal_pos,0,angle);
            kdTreeStart = new KdTree(true)
            {
                rootStart
            };
            kdTreeGoal = new KdTree(true)
            {
                rootGoal
            };

            my_path = new List<StateCar>();

        }

        private void FixedUpdate()
        {
            if (!(my_path[0] is null))
            {
                MoveCar();
                currentGoalPos.transform.position = my_path[0].pos;
            }
        }

        private void MoveCar()
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
            currentGoal = my_path[0];
            currentGoalPos.transform.position = currentGoal.pos;
            (float, float) inputs = GoTo(currentGoal.pos);
            m_Car.Move(inputs.Item2, inputs.Item1,inputs.Item1,0);
        }


        private (StateCar, StateCar) RRT(Vector3 xStart, Vector3 xGoal)
        {
            int i = 0;
            while (i < numberOfIterations)
            {
                i += 1;
                Vector3 xRand = SamplePointInMaze(true);
                StateCar xNew = Extend(rootStart, xRand, true);
                if (!(xNew is null) && Connect(xNew, out StateCar otherTreeState, false))
                {
                    return (xNew, otherTreeState);
                }
                StateCar xOtherTree;
                if (!(xNew is null))
                {
                    xOtherTree = Extend(rootGoal, xNew.pos, false);
                    if (!(xOtherTree is null) && Connect(xOtherTree, out otherTreeState, true))
                    {
                        return (otherTreeState, xOtherTree);
                    }
                }
                else
                {
                    Extend(rootGoal, xRand, false);
                }
            }
            return (null, null);
        }

        private StateCar Extend(StateCar treeRoot, Vector3 xRand, bool startTree)
        {
            //Sampling random pos in the maze
            //Finding nearest point of xRand in the graph
            StateCar xNearest = Nearest(xRand, startTree);
            StateCar newState = Steer(xNearest, xRand);
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


        private bool Connect(StateCar xNew, out StateCar otherTreeState,bool isStartTree)
        {
            otherTreeState = Nearest(xNew.pos, isStartTree);
            if (Vector3.Distance(otherTreeState.pos, xNew.pos) < validationDistance/2)
            {
                return true;
            }
            return false;
        }

        //private StateCar NewState(StateCar xNearest, float accel, float steering, float timeStep,float torque = 2000)
        //{
        //    (float, float, float,float, float) values = (xNearest.pos.x, xNearest.pos.z, xNearest.currentXSpeed,xNearest.currentYSpeed, xNearest.currentAngle);
        //    (float, float, float, float,float) derivatives = GetDerivatives(values, timeStep, accel, steering);
        //    Vector3 newPos = new Vector3(values.Item1 + derivatives.Item1*timeStep, 0, values.Item2 + derivatives.Item2*timeStep);
        //    float newXSpeed = values.Item3 + derivatives.Item3 * timeStep;
        //    float newYSpeed = values.Item4 + derivatives.Item4 * timeStep;
        //    float currentSpeed = Mathf.Sqrt(Mathf.Pow(newXSpeed, 2) + Mathf.Pow(newYSpeed, 2));
        //    float normalizedSpeed = Mathf.Clamp(currentSpeed, 0, m_Car.MaxSpeed);
        //    newXSpeed = normalizedSpeed * newXSpeed / currentSpeed;
        //    newYSpeed = normalizedSpeed * newYSpeed / currentSpeed;
        //    float newAngle = values.Item5 + derivatives.Item5* timeStep;
        //    return new StateCar(newPos, newXSpeed,newYSpeed, newAngle, accel, steering);
        //}

        //private (float,float,float,float,float) GetDerivatives((float, float, float,float, float) state,float timeStep, float accel, float steering,float torque = 2000)
        //{
        //    float currentSpeed = Mathf.Sqrt(Mathf.Pow(state.Item3, 2) + Mathf.Pow(state.Item4, 2));
        //    currentSpeed = Mathf.Sqrt(Mathf.Pow(currentSpeed + accel*torque*timeStep, 2) + Mathf.Pow(state.Item4+accel*torque*timeStep, 2));
        //    float angleDerivative = currentSpeed * Mathf.Tan(Mathf.Deg2Rad * steering * m_Car.m_MaximumSteerAngle) / carLength;
        //    return (currentSpeed*state.Item3* Mathf.Cos(state.Item5+angleDerivative*timeStep),
        //        currentSpeed * Mathf.Sin(state.Item5 + angleDerivative * timeStep),
        //        torque*accel*Mathf.Cos(state.Item5 + angleDerivative * timeStep) - angleDerivative*currentSpeed*Mathf.Sin(state.Item5 + angleDerivative * timeStep),
        //        torque * accel * Mathf.Sin(state.Item5 + angleDerivative * timeStep) + angleDerivative * currentSpeed * Mathf.Sin(state.Item5 + angleDerivative * timeStep),
        //        angleDerivative); //2000 is approximate torque
        //}

        private StateCar NewState(StateCar xNearest, float accel, float steering, float timeStep, float torque = 2000)
        {
            (float, float, float, float) values = (xNearest.pos.x, xNearest.pos.z, xNearest.currentSpeed, xNearest.currentAngle);
            (float, float) derivatives = GetDerivatives(values, timeStep, accel, steering);
            Vector3 newPos = new Vector3(values.Item1 + derivatives.Item1 * timeStep*Mathf.Cos(values.Item4), 0, values.Item2 + derivatives.Item1 * timeStep * Mathf.Sin(values.Item4));
            float newSpeed = xNearest.currentSpeed + derivatives.Item1 * timeStep;
            newSpeed = Mathf.Clamp(newSpeed, 0, m_Car.MaxSpeed);
            float newAngle = xNearest.currentAngle + derivatives.Item2* timeStep;
            return new StateCar(newPos, newSpeed,newAngle);
        }

        private (float, float) GetDerivatives((float, float, float, float) state, float timeStep, float accel, float steering, float torque = 2000)
        {
            return (accel * torque, state.Item3 * Mathf.Tan(Mathf.Deg2Rad * steering * m_Car.m_MaximumSteerAngle) / carLength);
        }


        private bool CollisionFree(Vector3 xStart,Vector3 xEnd)
        {
            //return (!Physics.Raycast(xStart, (xEnd - xStart).normalized, Vector3.Distance(xStart, xEnd), LayerMask.GetMask("Wall"))); //When the car is considered a point
            return !Physics.CheckBox((xStart + xEnd) / 2, new Vector3(carHalfWidth, 1, (xEnd - xStart).magnitude / 2 + carLength), Quaternion.LookRotation((xEnd - xStart).normalized, Vector3.up), LayerMask.GetMask("Wall")); // We consider the car as a box
        }

        private Vector3 SamplePointInMaze(bool isStartTree)
        {
            if (UnityEngine.Random.value <= 0.99)
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

        private StateCar Nearest(Vector3 xRand,bool isStartTree)
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
                return (StateCar) kdTreeStart.FindClosest(xRand);
            }
            return (StateCar) kdTreeGoal.FindClosest(xRand);
        }

        private StateCar Steer(StateCar xNearest,Vector3 xRand)
        {
            float distance = Mathf.Infinity;
            //float bestAngle = 0;
            StateCar finalState = new StateCar(Vector3.zero, 0,0);
            for (float steering=-1; steering<1.01;steering+=0.1f){
                for (float accel = -1;accel <1.01;accel += 0.1f)
                {
                    StateCar newState = NewState(xNearest, accel, steering, fixedDeltaTime);
                    float newDistance = Vector3.Distance(newState.pos, xRand);
                    if (newDistance< distance)
                    {
                        distance = newDistance;
                        finalState = newState;
                    }
                }
            }
            return finalState;
        }

        private (float, float) GoTo(Vector3 xGoal)
        {
            float distance = Mathf.Infinity;
            Vector3 projectedPosition = new Vector3(transform.position.x, 0, transform.position.z);
            float angle = Vector3.Angle(Vector3.right, transform.forward);
            if (angle < 0) { angle += 2 * Mathf.PI; }
            StateCar CurrentState = new StateCar(projectedPosition,m_Car.CurrentSpeed, angle);
            float finalAccel = 0;
            float finalSteering= 0;

            for (float steering = -1; steering < 1.01; steering += 0.05f)
            {
                for (float accel = -1; accel < 1.01; accel += 0.05f)
                {
                    StateCar newState = NewState(CurrentState, accel, steering, fixedDeltaTime);
                    float newDistance = Vector3.Distance(newState.pos, xGoal);
                    if (newDistance < distance)
                    {
                        distance = newDistance;
                        finalAccel = accel;
                        finalSteering= steering;
                    }
                }
            }
            return (finalAccel, finalSteering);
        }

    }
}
