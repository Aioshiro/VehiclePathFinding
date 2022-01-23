using System.Collections;
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

        [SerializeField] private List<Vector3> my_path;

        private Vector3 start_pos;
        private Vector3 goal_pos;
        private float terrainSize;
        private Vector2 terrainCenter;
        private float cMin;
        private (float, float, float, float) C;

        [Header("Informed RTT Settings")]
        [Tooltip("Eta for Steer Function")]
        [SerializeField] private float eta = 0.1f;
        [Tooltip("Radius in which we consider neighboors in the tree")]
        [SerializeField] private float rTT = 0.1f;
        [Tooltip("Radius where we consider we're close enough to the goal")]
        [SerializeField] private float sphereRadius = 1.0f;
        [SerializeField] private int NumberOfIterations = 500;
        [SerializeField] private float carLength;
        [SerializeField] private float carHalfWidth;

        private void Start()
        {

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            terrainCenter = new Vector2(terrain_manager.myInfo.x_high + terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high + terrain_manager.myInfo.z_low);
            terrainCenter /= 2;
            terrainSize = Mathf.Max(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low, terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low)/2;
            cMin = Vector3.Distance(start_pos, goal_pos);
            C = RotationToWorldFrame(start_pos, goal_pos);

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            (List<Vector3>, Dictionary<Vector3, Vector3>,Vector3) Graph = InformedRRTStar(NumberOfIterations, start_pos, goal_pos);

            //Recreating the tree
            foreach (var key in Graph.Item2.Keys)
            {
                Debug.DrawLine(key, Graph.Item2[key], Color.red,100f,false);
            }

            //Recreating the path
            Vector3 currentPoint = Graph.Item3;
            while (Graph.Item2.TryGetValue(currentPoint, out Vector3 parent))
            {
                Debug.DrawLine(currentPoint, parent, Color.green, 100f, false);
                currentPoint = parent;
            }

            
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            // this is how you access information about the terrain from the map
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                Debug.Log("Did Hit");
            }


            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);

        }

        private (List<Vector3>, Dictionary<Vector3, Vector3>) RRT(int N, Vector3 xStart, Vector3 xGoal)
        {
            ///Initializating graph
            List<Vector3> V = new List<Vector3>
            {
                xStart
            };
            Dictionary<Vector3, Vector3> parents = new Dictionary<Vector3, Vector3>();

            for (int i = 0; i < N; i++)
            {
                //Sampling random pos in the maze
                Vector3 xRand = SamplePointInMaze();
                //Finding nearest point of xRand in the graph
                Vector3 xNearest = Nearest(V, xRand);
                //Creating new Point between xNearest and xRand but close to xNearest
                Vector3 xNew = Steer(xNearest, xRand);
                if (CollisionFree(xNearest, xNew)) // If we can go to xNearest to xNew, we add xNew to the graph
                {
                    V.Add(xNew);
                    parents.Add(xNew, xNearest);
                }
            }
            return (V, parents);
        }

        private (List<Vector3>, Dictionary<Vector3, Vector3>) RRTStar(int N, Vector3 xStart, Vector3 xGoal)
        {
            ///Initializating graph
            List<Vector3> V = new List<Vector3>
            {
                xStart
            };
            Dictionary<Vector3, Vector3> parents = new Dictionary<Vector3, Vector3>();
            for (int i = 0; i < N; i++)
            {
                //Sampling random pos in the maze
                Vector3 xRand = SamplePointInMaze();
                //Finding nearest point of xRand in the graph
                Vector3 xNearest = Nearest(V, xRand);
                //Creating new Point between xNearest and xRand but close to xNearest
                Vector3 xNew = Steer(xNearest, xRand);

                if (CollisionFree(xNearest, xNew)) 
                {
                    // If we can go to xNearest to xNew, we add xNew to the graph and link it to the closest neighboor
                    List<Vector3> XNear = Near(V, xNew, rTT);
                    V.Add(xNew);
                    Vector3 xMin = xNearest;
                    float cMin = Vector3.Distance(xNearest, start_pos) + Vector3.Distance(xNearest, xNew);
                    foreach (Vector3 xNear in XNear)
                    {
                        float cNew = Vector3.Distance(xNear, start_pos) + Vector3.Distance(xNear, xNew);
                        if (cNew < cMin && CollisionFree(xNear, xNew))
                        {
                            xMin = xNear;
                            cMin = cNew;
                        }
                    }
                    parents.Add(xNew, xMin);
                    foreach (Vector3 xNear in XNear)
                    {
                        float cNew = Vector3.Distance(xNew, start_pos) + Vector3.Distance(xNear, xNew);
                        if (cNew < Vector3.Distance(xNear, start_pos) && CollisionFree(xNear, xNew))
                        {
                            parents[xNew] = xNear;
                        }
                    }
                }
            }
            return (V, parents);
        }

        private (List<Vector3>, Dictionary<Vector3, Vector3>,Vector3) InformedRRTStar(int N, Vector3 xStart, Vector3 xGoal)
        {
            ///Initializating variables
            List<Vector3> V = new List<Vector3>
            {
                xStart
            };
            Dictionary<Vector3, Vector3> parents = new Dictionary<Vector3, Vector3>();
            List<Vector3> XsoIn = new List<Vector3>();
            float cBest;
            for (int i = 0; i < N; i++)
            {
                //Calculating min(costs(XsoIn)), if it's empty, return +inf
                if (XsoIn.Count == 0)
                {
                    cBest = Mathf.Infinity;
                }
                else
                {
                    List<float> costs = new List<float>();
                    foreach (Vector3 x in XsoIn)
                    {
                        costs.Add(x.magnitude);
                    }
                    cBest = Mathf.Min(costs.ToArray());
                }
                //Sampling random pos in the maze
                Vector3 xRand = Sample(cBest);
                //Finding nearest point of xRand in the graph
                Vector3 xNearest = Nearest(V, xRand);
                //Creating new Point between xNearest and xRand but close to xNearest
                Vector3 xNew = Steer(xNearest, xRand);

                if (CollisionFree(xNearest,xNew))
                {
                    // If we can go to xNearest to xNew, we add xNew to the graph and link it to the closest neighboor
                    List<Vector3> XNear = Near(V, xNew, rTT);
                    V.Add(xNew);
                    Vector3 xMin = xNearest;
                    float cMin = Vector3.Distance(xNearest, start_pos) + Vector3.Distance(xNearest, xNew);
                    foreach (Vector3 xNear in XNear)
                    {
                        float cNew = Vector3.Distance(xNear, start_pos) + Vector3.Distance(xNear, xNew);
                        if (cNew < cMin && CollisionFree(xNear,xNew))
                        {
                            xMin = xNear;
                            cMin = cNew;
                        }
                    }
                    parents[xNew] = xMin;
                    foreach (Vector3 xNear in XNear)
                    {
                        float cNew = Vector3.Distance(xNew, start_pos) + Vector3.Distance(xNear, xNew);
                        if (cNew < Vector3.Distance(xNear, start_pos) &&CollisionFree(xNear,xNew))
                        {
                            parents[xNew] = xNear;
                        }
                    }
                    if (Physics.CheckSphere(xNew, sphereRadius, LayerMask.GetMask("Goal")))
                    {
                        //If xNew in near goal, that means we found an admissible path !
                        XsoIn.Add(xNew);
                    }
                }
            }
            Vector3 endPoint = new Vector3();
            float costMin = Mathf.Infinity;
            foreach (Vector3 x in XsoIn)
            {
                if (Vector3.Distance(x, xStart) < costMin)
                {
                    costMin = Vector3.Distance(x, xStart);
                    endPoint = x;
                }
            }
            return (V, parents,endPoint);
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
                xRand += new Vector3(terrainCenter.x, 0, terrainCenter.y);
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

        private Vector3 Nearest(List<Vector3> V,Vector3 xRand)
        {
            Vector3 xMin = new Vector3();
            float minDist = Mathf.Infinity;
            foreach (Vector3 x in V)
            {
                float temp = Vector3.Distance(x, xRand);
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
            return eta * z+xNearest;
        }

        private List<Vector3> Near(List<Vector3> V, Vector3 xNew, float rRTT)
        {
            List<Vector3> XNear = new List<Vector3>();
            foreach (Vector3 x in V)
            {
                if (Vector3.Distance(x, xNew) < rRTT)
                {
                    XNear.Add(x);
                }
            }
            return XNear;
        }
    }
}
