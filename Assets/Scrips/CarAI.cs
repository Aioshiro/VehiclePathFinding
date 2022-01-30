﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        [SerializeField] private List<(Vector3, float, float)> my_path;

        private Vector3 start_pos;
        private Vector3 goal_pos;
        private float terrainSize;
        private Vector2 terrainCenter;
        private float cMin;
        private (float, float, float, float) C;
        private Dictionary<(Vector3, float, float), float> costs;
        private bool arrivedAtGoal = false;
        private int goalIndex = 0;

        [Header("Informed RRT Settings")]
        [Tooltip("Eta for Steer Function")]
        [SerializeField] private float steeringRadius = 1f;
        [Tooltip("Radius in which we consider neighboors in the tree")]
        [SerializeField] private float neighborsRadius = 1f;
        [Tooltip("Radius where we consider we're close enough to the goal")]
        [SerializeField] private float goalRadius = 1.0f;
        [SerializeField] private int NumberOfIterations = 500;
        [SerializeField] private float carLength;
        [SerializeField] private float carHalfWidth;

        [Header("Car Movement Parameters")]
        [Tooltip("Max distance where we considered the car passed through the point")]
        [SerializeField] private float validatedDistance;


        private void Start()
        {

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            terrainCenter = new Vector2(terrain_manager.myInfo.x_high + terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high + terrain_manager.myInfo.z_low);
            terrainCenter /= 2;
            terrainSize = Mathf.Max(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low)/2;
            cMin = Vector3.Distance(start_pos, goal_pos);
            C = RotationToWorldFrame(start_pos, goal_pos);
            costs = new Dictionary<(Vector3, float, float), float>();


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            (List<(Vector3, float, float)>, Dictionary<(Vector3, float, float), (Vector3, float, float)>, (Vector3, float, float)) Graph = RRT(NumberOfIterations, start_pos, goal_pos);

            //Recreating the tree
            foreach (var key in Graph.Item2.Keys)
            {
                Debug.DrawLine(key.Item1, Graph.Item2[key].Item1, Color.red,100f,false);
            }

            my_path = new List<(Vector3, float, float)>();

            //Recreating the path
            (Vector3, float, float) currentPoint = Graph.Item3;
            my_path.Add(currentPoint);
            while (Graph.Item2.TryGetValue(currentPoint, out (Vector3, float, float) parent))
            {
                my_path.Add(parent);
                Debug.DrawLine(currentPoint.Item1, parent.Item1, Color.green, 100f, false);
                currentPoint = parent;
            }
            my_path.Reverse();
            goalIndex = 0;

        }


        private void FixedUpdate()
        {

            //// this is how you access information about the terrain from the map
            //int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            //int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            //float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            //float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            //// this is how you access information about the terrain from a simulated laser range finder
            //RaycastHit hit;
            //float maxRange = 50f;
            //if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            //{
            //    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            //    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
            //    Debug.Log("Did Hit");
            //}

            //// this is how you control the car
            //m_Car.Move(1f, 1f, 1f, 0f);
            if (my_path.Count < goalIndex)
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
            (Vector3 oldPos, float speed, float angle) = my_path[goalIndex];
            (Vector3 goal, float newSpeed, float newAngle) = my_path[goalIndex + 1];
            goalIndex += 1;
            float steering = Mathf.Rad2Deg*Mathf.Atan((newAngle - angle) * carLength / (newSpeed * Time.fixedDeltaTime))/m_Car.m_MaximumSteerAngle;
            float accel = (newSpeed - speed)/Time.fixedDeltaTime;
            m_Car.Move(steering, accel, accel, 0.0f);
        }



        private (List<(Vector3,float,float)>, Dictionary<(Vector3, float, float), (Vector3, float, float)>, (Vector3,float,float)) RRT(int N, Vector3 xStart, Vector3 xGoal)
        {
            ///Initializating graph
            List<(Vector3, float, float)> V = new List<(Vector3, float, float)>
            {
                (xStart,0,90)
            };
            costs.Add((xStart, 0, 90), 0);
            Dictionary<(Vector3, float, float), (Vector3, float, float)> parents = new Dictionary<(Vector3, float, float), (Vector3, float, float)>();
            bool foundPath = false;
            int i = 0;
            float timeStep = Time.fixedDeltaTime;
            (Vector3,float,float) finalPoint = (new Vector3(), new float(), new float());
            while (i<N && !foundPath)
            {
                i += 1;
                //Sampling random pos in the maze
                Vector3 xRand = SamplePointInMaze();
                //Finding nearest point of xRand in the graph
                (Vector3, float, float) xNearest = Nearest(V, xRand);
                (float accel, float steering) = SelectInput(xNearest, xRand);
                //Creating new Point between xNearest and xRand but close to xNearest
                (Vector3,float,float) xNew = NewState(xNearest, accel,steering,timeStep);
                if (CollisionFree(xNearest.Item1, xNew.Item1)) // If we can go to xNearest to xNew, we add xNew to the graph
                {
                    List<(Vector3, float, float)> XNear = Near(V, xNew, neighborsRadius);
                    V.Add(xNew);
                    (Vector3, float, float) xMin = xNearest;
                    float cMin = Cost(xNearest,parents) + CostLine(xNearest.Item1, xNew.Item1);
                    foreach (var xNear in XNear)
                    {
                        if (CollisionFree(xNear.Item1, xNew.Item1)&& Cost(xNear,parents)+CostLine(xNear.Item1, xNew.Item1)< cMin)
                        {
                            xMin = xNear;
                            cMin = Cost(xNear, parents) + CostLine(xNear.Item1, xNew.Item1);
                        }
                    }
                    parents.Add(xNew, xMin);
                    foreach (var xNear in XNear)
                    {
                        if (CollisionFree(xNear.Item1, xNew.Item1) && Cost(xNew, parents) + CostLine(xNear.Item1, xNew.Item1) < Cost(xNear, parents)){
                            parents[xNear] = xNew;
                        }
                    }
                    if (Physics.CheckSphere(xNew.Item1, goalRadius, LayerMask.GetMask("Goal")))
                    {
                        //If xNew in near goal, that means we found an admissible path !
                        foundPath = true;
                        finalPoint = xNew;
                    }
                }
            }
            return (V, parents,finalPoint);
        }

        //private (float,float) SelectInput(Vector3 xNear,Vector3 xRand)
        //{
        //    float accel = (0.1f-properties[xNear].Item1)/Time.fixedDeltaTime;
        //    float currentCarAngle = properties[xNear].Item2;
        //    float desiredCarAngle = Vector3.SignedAngle(Vector3.right, xRand - xNear, Vector3.up);
        //    //Setting the angles from [-180,180] to [0,360] for continuity
        //    if (currentCarAngle < 0)
        //    {
        //        currentCarAngle = 360 + currentCarAngle;
        //    }
        //    if (desiredCarAngle < 0)
        //    {
        //        desiredCarAngle = 360 + desiredCarAngle;
        //    }
        //    float steering = Mathf.Rad2Deg * Mathf.Atan(Mathf.Deg2Rad * (desiredCarAngle - currentCarAngle) * carLength / (properties[xNear].Item1 * Time.fixedDeltaTime)) / m_Car.m_MaximumSteerAngle;
        //    properties.Add(xRand, (accel, steering));
        //    return (accel,steering);
        //}
        private (float, float) SelectInput((Vector3, float, float) xNear, Vector3 xRand)
        {
            return (2*UnityEngine.Random.value-1, UnityEngine.Random.value);
        }


        private (Vector3,float,float) NewState((Vector3, float, float) xNearest,float accel, float steering,float timeStep)
        {
            float speed = xNearest.Item2;
            float angle = xNearest.Item3;
            float newAngle = angle + speed * timeStep * Mathf.Tan(steering * m_Car.m_MaximumSteerAngle) / carLength;
            float newSpeed = speed + accel * timeStep;
            Vector3 newPos = new Vector3(xNearest.Item1.x + timeStep * newSpeed * Mathf.Cos(newAngle), 0, xNearest.Item1.z + timeStep* newSpeed * Mathf.Sin(newAngle));
            return (newPos,newSpeed,newAngle);
        }

        private float CostLine(Vector3 xA,Vector3 xB)
        {
            return Vector3.Distance(xA,xB);
        }

        private float Cost((Vector3,float,float) x,Dictionary<(Vector3,float,float),(Vector3,float,float)> parents)
        {
            Debug.Log(x);
            if (costs.TryGetValue(x,out float cost)){
                return cost;
            }
            if (parents.TryGetValue(x, out var parent))
            {
                cost = Vector3.Distance(x.Item1, parent.Item1) + Cost(parent, parents);
            }
            costs.Add(x, cost);
            return cost;
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

        private Vector3 Sample(float cMax)
        {
            Vector3 xRand;
            if (cMax < Mathf.Infinity)
            {
                float r1 = cMax / 2;
                float r2 = Mathf.Sqrt(cMax * cMax - cMin * cMin) / 2;
                Vector2 xBall = UnityEngine.Random.insideUnitCircle;
                xRand = new Vector3(C.Item1 * r1 * xBall.x + C.Item2 * r2 * xBall.y,0, C.Item3 * r1 * xBall.x + C.Item4 * r2 * xBall.y);
                xRand += new Vector3(start_pos.x/2+goal_pos.x/2, 0, start_pos.z / 2 + goal_pos.z / 2);
            }
            else
            {
                xRand = SamplePointInMaze();
            }
            return new Vector3(xRand.x, 0, xRand.z);
        }

        private (float, float, float, float) RotationToWorldFrame(Vector3 xStart, Vector3 xGoal)
        {
            Vector3 a = xGoal - xStart;
            a.Normalize();
            Vector2 a1 = new Vector2(a.x, a.z);
            //Calculating SVD of ((a1.x 0),(a1.y 0))
            float phi =  Mathf.Atan2(a1.y/2,a1.x/2);
            return (Mathf.Cos(phi),
                -Mathf.Sin(phi),
                Mathf.Sin(phi),
                Mathf.Cos(phi)
                );

        }

        private (Vector3, float, float) Nearest(List<(Vector3,float,float)> V,Vector3 xRand)
        {
            (Vector3, float, float) xMin = (new Vector3(), new float(), new float());
            float minDist = Mathf.Infinity;
            foreach ((Vector3,float,float) x in V)
            {
                float temp = Vector3.Distance(x.Item1, xRand);
                if (temp < minDist)
                {
                    minDist = temp;
                    xMin = x;
                }
            }
            return xMin;
        }

        private Vector3 Steer(Vector3 xNearest,Vector3 xRand)
        {
            Vector3 z = xRand - xNearest;
            z.Normalize();
            return steeringRadius * z+xNearest;
        }

        private List<(Vector3,float,float)> Near(List<(Vector3,float,float)> V, (Vector3,float,float) xNew, float rRTT)
        {
            List<(Vector3,float,float)> XNear = new List<(Vector3,float,float)>();
            foreach ((Vector3,float,float) x in V)
            {
                if (Vector3.Distance(x.Item1, xNew.Item1) < rRTT)
                {
                    XNear.Add(x);
                }
            }
            return XNear;
        }
    }
}
