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
        [SerializeField] private List<State> my_path;

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
        [SerializeField] private float goalRadius = 1.0f;
        [SerializeField] private int numberOfIterations = 500;
        [SerializeField] private float carLength;
        [SerializeField] private float carHalfWidth;
        private State currentGoal = new State(Vector3.zero,0,0,0,0);
        public Vector3 currentGoalPos;
        private List<State> V;
        private State root;

        private class State
        {
            public Vector3 pos;
            private float _cost;
            public float currentSpeed;
            public float currentAngle;
            public float accelFromLastState;
            public float steeringFromLastState;
            public State parent;
            public List<State> children;

            public State(Vector3 pos,float currentSpeed,float currentAngle, float accelFromLastState, float steeringFromLastState)
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



        private void Start()
        {
            InitializeRRTParameters();

            // get the car controller
            m_Car = GetComponent<CarController>();


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            State finalPoint = RRT(start_pos, new Vector3(150,0,175));

            ShowTree(root);
            ShowingTreeAndPath(finalPoint);

        }

        private void ShowingTreeAndPath(State finalState)
        {
            //Recreating the path
            State currentState = finalState;
            my_path.Add(currentState);
            while (!(currentState.parent is null))
            {
                my_path.Add(currentState.parent);
                Debug.DrawLine(currentState.pos, currentState.parent.pos, Color.green, Mathf.Infinity, false);
                currentState= currentState.parent;
            }
            my_path.Reverse();
        }

        private void ShowTree(State currentState)
        {
            if (currentState.children.Count != 0)
            {
                foreach( State child in currentState.children)
                {
                    Debug.DrawLine(currentState.pos, child.pos, Color.red, Mathf.Infinity, false);
                    ShowTree(child);
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

            //Relative to RRT
            fixedDeltaTime = Time.fixedDeltaTime;
            root = new State(start_pos, 0, Mathf.Deg2Rad * 90, 0, 0);

            V = new List<State>
            {
                root
            };
            my_path = new List<State>();

        }

        private void FixedUpdate()
        {
            if (my_path.Count > 0)
            {
                MoveCar();
            }

        }
        private void MoveCar()
        {
            if (arrivedAtGoal)
            {
                m_Car.Move(0, 0, 0, 0);
            }
            Vector3 projectionPosition = new Vector3(transform.position.x, 0, transform.position.z);
            while(Vector3.Distance(my_path[0].pos, projectionPosition) < validationDistance)
            {
                my_path.RemoveAt(0);
            }
            currentGoal = my_path[0];
            currentGoalPos = currentGoal.pos;
            float accel = (currentGoal.currentSpeed - m_Car.CurrentSpeed) / fixedDeltaTime;
            float currentAngle = Vector3.SignedAngle(Vector3.right, Vector3.forward,Vector3.up)*Mathf.Deg2Rad;
            if (currentAngle < 0)
            {
                currentAngle += 2 * Mathf.PI;
            }
            float steering = Mathf.Rad2Deg*Mathf.Atan((currentGoal.currentAngle-currentAngle) * carLength / (m_Car.CurrentSpeed * fixedDeltaTime))/m_Car.m_MaximumSteerAngle;
            m_Car.Move(steering, accel, accel, 0);
        }





        private State RRT(Vector3 xStart, Vector3 xGoal)
        {
            ///Initializating graph
            bool foundPath = false;
            int i = 0;
            State finalPoint = new State(Vector3.zero, 0, 0, 0, 0);
            while (i<numberOfIterations && !foundPath)
            {
                i += 1;
                //Sampling random pos in the maze
                Vector3 xRand = SamplePointInMaze();
                //Finding nearest point of xRand in the graph
                State xNearest = Nearest(V, xRand);
                State newState = Steer(xNearest, xRand);
                if (CollisionFree(xNearest.pos, newState.pos)) // If we can go to xNearest to xNew, we add xNew to the graph
                {
                    V.Add(newState);
                    newState.parent = xNearest;
                    xNearest.children.Add(newState);
                    if (Vector3.Distance(newState.pos,xGoal)<goalRadius)
                    {
                        //If xNew in near goal, that means we found an admissible path !
                        foundPath = true;
                        finalPoint = newState;
                    }
                    
                }
                
            }
            return finalPoint;
        }

        private State NewState(State xNearest, float accel, float steering, float timeStep)
        {
            (float, float, float, float) values = (xNearest.pos.x, xNearest.pos.z, xNearest.currentSpeed, xNearest.currentAngle);
            (float, float, float, float) derivatives = GetDerivatives(values, timeStep, accel, steering);
            Vector3 newPos = new Vector3(values.Item1 + derivatives.Item1*timeStep, 0, values.Item2 + derivatives.Item2*timeStep);
            float newSpeed = values.Item3 + derivatives.Item3 * timeStep;
            float newAngle = values.Item4 + derivatives.Item4 * timeStep;
            return new State(newPos, newSpeed, newAngle, accel, steering);
        }

        private (float,float,float,float) GetDerivatives((float, float, float, float) state,float timeStep, float accel, float steering)
        {
            float angleDerivative = 2500 * accel * timeStep * Mathf.Tan(Mathf.Deg2Rad * steering * m_Car.m_MaximumSteerAngle) / carLength;
            return (2500 * accel *timeStep* Mathf.Cos(state.Item4+angleDerivative*timeStep), 2500 * accel * timeStep * Mathf.Sin(state.Item4 + angleDerivative * timeStep),2500*accel, angleDerivative); //2000 is approximate torque
        }

        private bool CollisionFree(Vector3 xStart,Vector3 xEnd)
        {
            //return (!Physics.Raycast(xStart, (xEnd - xStart).normalized, Vector3.Distance(xStart, xEnd), LayerMask.GetMask("Wall"))); //When the car is considered a point
            return !Physics.CheckBox((xStart+xEnd)/2, new Vector3(carHalfWidth,1,(xEnd-xStart).magnitude/2+carLength), Quaternion.LookRotation((xEnd-xStart).normalized,Vector3.up), LayerMask.GetMask("Wall")); // We consider the car as a box
        }

        private Vector3 SamplePointInMaze()
        {
            Vector3 xRand = new Vector3(2 * UnityEngine.Random.value - 1, 0, 2 * UnityEngine.Random.value - 1);
            xRand *= terrainSize; //Sampling random pos in the maze
            xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
            return xRand;
        }

        private State Nearest(List<State> V,Vector3 xRand)
        {
            State xMin = new State(Vector3.zero, 0, 0, 0, 0);
            float minDist = Mathf.Infinity;
            foreach (State x in V)
            {
                float temp = Vector3.Distance(x.pos, xRand);
                if (temp < minDist)
                {
                    minDist = temp;
                    xMin = x;
                }
            }
            return xMin;
        }

        private State Steer(State xNearest,Vector3 xRand)
        {
            float distance = Mathf.Infinity;
            //float bestAngle = 0;
            State finalState = new State(Vector3.zero, 0, 0, 0, 0);
            for (float steering=-1; steering<=1;steering+=0.1f){
                for (float accel = 0;accel <=0.5;accel += 0.1f)
                {
                    State newState = NewState(xNearest, accel, steering, fixedDeltaTime);
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

    }
}
