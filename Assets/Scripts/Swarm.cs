using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;
    public int numberOfBoids = 200;
    public float boidForceScale = 20f;          
    public float maxSpeed = 5.0f;
    public float rotationSpeed = 40.0f;
    public float obstacleCheckRadius = 1.0f;
    public float separationWeight = 1.1f;       
    public float alignmentWeight = 0.5f;        
    public float cohesionWeight = 1f;           
    public float goalWeight = 1f;               
    public float obstacleWeight = 0.9f;         
    public float wanderWeight = 0.3f;           
    public float neighbourDistance = 2.0f;
    public float initializationRadius = 1.0f;
    public float initializationForwardRandomRange = 50f;
    private BBoid[] boids;
    private Transform[] boidObjects;
    private float sqrNeighbourDistance;
    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        boidZeroPath = new NavMeshPath();
        InitBoids();
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];


        // Initialize each boid
        for (int i = 0; i < numberOfBoids; i++)
        {
            // Random position inside a sphere around the Swarm object
            Vector3 randomPos = Random.insideUnitSphere * initializationRadius;
            Vector3 pos = transform.position + randomPos;

            // Keep Y within world bounds [1,4]
            pos.y = Mathf.Clamp(pos.y, 1f, 4f);

            // Random horizontal heading (yaw) within ±initializationForwardRandomRange
            float yaw = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
            Quaternion randomRotation = Quaternion.Euler(0f, yaw, 0f);
            Vector3 forward = (randomRotation * Vector3.forward).normalized;

            // Initial velocity along forward direction
            Vector3 velocity = forward * (maxSpeed * 0.5f);

            boids[i].position = pos;
            boids[i].forward = forward;
            boids[i].velocity = velocity;
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;

            Transform obj = Instantiate(boidPrefab, pos, Quaternion.LookRotation(forward));
            boidObjects[i] = obj;
        }
    }

    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {

        // Reset all boid rule vectors and total force
        if (boids == null) 
        {
            return;
        }

        //
        int boidCount = boids.Length;

        //Loop through all boids and reset their forces
        for (int i = 0; i < boidCount; i++)
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
        }
    }

    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0)
        {
            return;
        }

        float dt = Time.fixedDeltaTime;

        //Resetting values at the top of the simulation loop
        ResetBoidForces();

        int boidCount = boids.Length;

        // looping through all boids to calculate forces and update positions
        for (int i = 0; i < boidCount; i++)
        {
            // neighbour information
            List<int> neighbors = new List<int>();
            Vector3 separationSum = Vector3.zero;
            Vector3 alignmentSum = Vector3.zero;
            Vector3 cohesionSum = Vector3.zero;

            //looping through all boids to find neighbors
            for (int j = 0; j < boidCount; j++)
            {
                // if the boid is itself, skip
                if (i == j) 
                {
                    continue;
                }

                // check distance to other boid
                Vector3 toNeighbor = boids[j].position - boids[i].position;
                float sqrDist = toNeighbor.sqrMagnitude;

                // if outside neighbour distance or too close, skip
                if (sqrDist > sqrNeighbourDistance || sqrDist < 0.00001f)
                {
                    continue;
                }


                // check if in front using FOV
                Vector3 dirToNeighbor = toNeighbor.normalized;

                // if behind, skip
                if (Vector3.Dot(dirToNeighbor, boids[i].forward) <= 0f)
                {
                    continue;
                }

                // it's a neighbour
                neighbors.Add(j);

            
                separationSum += (-dirToNeighbor);

         
                alignmentSum += boids[j].velocity;

       
                cohesionSum += boids[j].position;
            }

            // if we have neighbors, compute rule vectors
            if (neighbors.Count > 0)
            {
                // separation, alignment, cohesion calculations


                boids[i].separation = separationSum.normalized;

 
                Vector3 avgVel = alignmentSum / neighbors.Count;

                // if average velocity is significant, normalize it
                if (avgVel.sqrMagnitude > 0.000001f)
                {
                    boids[i].alignment = avgVel.normalized;
                }
                else
                {
                    boids[i].alignment = Vector3.zero;
                }


                Vector3 centerOfMass = cohesionSum / neighbors.Count;
                Vector3 toCenter = centerOfMass - boids[i].position;

                // if toCenter is significant, normalize it
                if (toCenter.sqrMagnitude > 0.000001f)
                {
                    boids[i].cohesion = toCenter.normalized;
                }
                else
                {
                    boids[i].cohesion = Vector3.zero;
                }
            }
            else
            {

                boids[i].separation = Vector3.zero;
                boids[i].alignment = Vector3.zero;
                boids[i].cohesion = Vector3.zero;
            }



            Collider[] obstacles = Physics.OverlapSphere(boids[i].position, obstacleCheckRadius);
            Vector3 obstacleAvoidance = Vector3.zero;

            // loop through obstacles to compute avoidance vector
            for (int k = 0; k < obstacles.Length; k++)
            {
                Collider obstacle = obstacles[k];

                // Ignore self collision with own boid object
                if (boidObjects != null && boidObjects.Length > i && boidObjects[i] != null)
                {
                    if (obstacle.gameObject == boidObjects[i].gameObject ||
                        obstacle.transform.IsChildOf(boidObjects[i]))
                    {
                        continue;
                    }
                }


                Vector3 closestPoint = obstacle.ClosestPoint(boids[i].position);
                Vector3 normal = boids[i].position - closestPoint;

                // if not an obstacle, skip
                if (normal.sqrMagnitude > 0.00001f)
                {
                    obstacleAvoidance += normal.normalized;
                }
            }


            Vector3 boundaryForce = Vector3.zero;

            // World bounds check (cube: x[-8,8], z[-8,8], y[1,4])
            if (boids[i].position.x > 8f)
                boundaryForce += new Vector3(-1f, 0f, 0f);

            // Left boundary
            if (boids[i].position.x < -8f)
                boundaryForce += new Vector3(1f, 0f, 0f);

            // Front boundary
            if (boids[i].position.z > 8f)
                boundaryForce += new Vector3(0f, 0f, -1f);

            // Back boundary
            if (boids[i].position.z < -8f)
                boundaryForce += new Vector3(0f, 0f, 1f);

            //  Top boundary
            if (boids[i].position.y > 4f)
                boundaryForce += new Vector3(0f, -1f, 0f);

            // Bottom boundary
            if (boids[i].position.y < 1f)
                boundaryForce += new Vector3(0f, 1f, 0f);

            Vector3 obstacleTotal = obstacleAvoidance + boundaryForce;

            // if there's an obstacle avoidance vector, normalize it
            if (obstacleTotal.sqrMagnitude > 0.00001f)
            {
                boids[i].obstacle = obstacleTotal.normalized;
            }
            else
            {
                boids[i].obstacle = Vector3.zero;
            }


            Vector3 totalForce = Vector3.zero;

            bool hasNeighbours = (boids[i].separation != Vector3.zero) || (boids[i].alignment != Vector3.zero) || (boids[i].cohesion != Vector3.zero);

            // if we have neighbours, apply the three main boid rules
            if (boids[i].separation != Vector3.zero)
            {
                totalForce += separationWeight * (boids[i].separation * boidForceScale - boids[i].velocity);
            }

            // if we have neighbours, apply alignment rule
            if (boids[i].alignment != Vector3.zero)
            {
                totalForce += alignmentWeight * (boids[i].alignment * boidForceScale - boids[i].velocity);
            }

            // if we have neighbours, apply cohesion rule
            if (boids[i].cohesion != Vector3.zero)
            {
                totalForce += cohesionWeight * (boids[i].cohesion * boidForceScale - boids[i].velocity);
            }

            // if no neighbours, apply wander rule
            if (hasNeighbours == false)
            {
                Vector3 wanderDirection;


                // determine wander direction based on current velocity or forward
                if (boids[i].velocity.sqrMagnitude > 0.00001f)
                {
                    wanderDirection = boids[i].velocity.normalized;
                }
                else
                {
                    wanderDirection = boids[i].forward;
                }

                totalForce += wanderWeight * (wanderDirection * boidForceScale - boids[i].velocity);
            }


            // if there's an obstacle, apply obstacle avoidance rule
            if (boids[i].obstacle != Vector3.zero)
            {
                totalForce += obstacleWeight * (boids[i].obstacle * boidForceScale - boids[i].velocity);
            }

            // if boid zero, and navigating toward goal, apply goal following rule
            if (i == 0 &&
                boidZeroNavigatingTowardGoal &&
                boidZeroPath != null &&
                boidZeroPath.status == NavMeshPathStatus.PathComplete &&
                boidZeroPath.corners.Length > 1)
            {
                NavMeshHit hit;
                if (NavMesh.SamplePosition(boids[0].position, out hit, 2f, NavMesh.AllAreas))
                {
                    // If close enough to current corner, advance
                    if (Vector3.Distance(hit.position, boidZeroPath.corners[currentCorner]) < 1f)
                    {
                        currentCorner++;

                        // if reached the end of the path, stop navigating
                        if (currentCorner >= boidZeroPath.corners.Length)
                        {
                           
                            boidZeroPath.ClearCorners();
                            currentCorner = 0;
                            boidZeroNavigatingTowardGoal = false;
                        }
                    }

                    // If still navigating, apply goal following rule
                    if (boidZeroNavigatingTowardGoal && currentCorner < boidZeroPath.corners.Length)
                    {
                        Vector3 goalDirection =
                            (boidZeroPath.corners[currentCorner] - boids[0].position).normalized;

                        totalForce += goalWeight *
                                      (goalDirection * boidForceScale - boids[0].velocity);
                    }
                }
            }

            boids[i].currentTotalForce = totalForce;
        }

        // loop through all boids to update velocity and position
        for (int i = 0; i < boidCount; i++)
        {

            boids[i].velocity += boids[i].currentTotalForce * dt;


            float speed = boids[i].velocity.magnitude;

            // if speed exceeds maxSpeed, clamp it
            if (speed > maxSpeed)
                boids[i].velocity = boids[i].velocity / speed * maxSpeed;

  
            boids[i].position += boids[i].velocity * dt;

            // if velocity is significant, update forward direction
            if (boids[i].velocity.sqrMagnitude > 0.01f)
            {
                Vector3 targetForward = boids[i].velocity.normalized;

                Quaternion currentRot = Quaternion.LookRotation(boids[i].forward);
                Quaternion targetRot = Quaternion.LookRotation(targetForward);
                float maxDegreesDelta = rotationSpeed * dt;
                Quaternion newRot = Quaternion.RotateTowards(currentRot, targetRot, maxDegreesDelta);
                boids[i].forward = newRot * Vector3.forward;
            }

            // if we have a boid object, update its position and rotation
            if (boidObjects != null && boidObjects[i] != null)
            {
                boidObjects[i].position = boids[i].position;
                if (boids[i].forward.sqrMagnitude > 0.01f)
                    boidObjects[i].rotation = Quaternion.LookRotation(boids[i].forward);
            }
        }
    }

    private void Update()
    {
        /* Optional debug for boidzero (left commented out for assignment)
        if (boids == null || boids.Length == 0) return;

        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec.normalized, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }

        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        */
    }


    ///  <summary>
    /// Set a new goal for boid zero to navigate toward
    /// </ summary>
    public void SetGoal(Vector3 goal)
    {
        // if already navigating, ignore new goal
        if (boidZeroNavigatingTowardGoal)
        { 
        return;
        }

        // if no boids, return
        if (boids == null || boids.Length == 0)
        {
            return;
        }

        boidZeroGoal = goal;


        // Initialize navmesh path if null
        if (boidZeroPath == null)
        {
            boidZeroPath = new NavMeshPath();
        }

        // Nearest navmesh point to boidzero
        NavMeshHit startHit;

        // if boid zero not on navmesh, return
        if (NavMesh.SamplePosition(boids[0].position, out startHit, 5f, NavMesh.AllAreas) == false)
        {
            return;
        }
        
        NavMeshHit goalHit;

        // if goal not on navmesh, return
        if (NavMesh.SamplePosition(goal, out goalHit, 5f, NavMesh.AllAreas) == false)
        {
            return;
        }

        // if start and goal are the same, return
        if (NavMesh.CalculatePath(startHit.position, goalHit.position, NavMesh.AllAreas, boidZeroPath))
        {
            // if path is valid, start navigating
            if (boidZeroPath.status == NavMeshPathStatus.PathComplete &&
                boidZeroPath.corners.Length > 1)
            {
                currentCorner = 1;
                boidZeroNavigatingTowardGoal = true;
            }
        }
    }
}