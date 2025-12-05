# AISystems

I have successfully completed everything in the 

[2] Correct and automatic initialization of the boids. All boids should be randomly initialized within the initialization radius with a random forward heading within the forward random range.

 - Completed this successfully
 - Implemented in InitBOids() using Random.insideUnitSphere * initializationRadius + transform.position


[3] Correct handling of the resetting of values at the top of the simulation loop

 - Completed this successfully
 - Implemented in ResetBoidForces() and is called at the top of FixedUpdate()

[3] Correct implementation of building the neighbour list, by simulating vision using distance and dot product.
 - Completed this successfully
 - Implemented inside FixedUpdate() 


[3] Correct implementation of separation rule
 - Completed this successfully
 - For each neighbour seperationSum accumated the direction away from neighbout using separationSum += (-dirToNeighbor);
 - When neighbours exist then boids[i].separation = separationSum.normalized;



[3] Correct implementation of alignment rule
 - Completed this successfully
 - Neghbour velocities are added together in alignmentSum
 - Avg vel is Vector3 avgVel = alignmentSum / neighbors.Count;



[3] Correct implementation of cohesion rule
 - Completed this successfully
 - Neighbour positions are added together in cohesianSum
 - Cente of mass: centerOfMass = cohesionSum / neighbors.Count;
 - Vector toward centre: toCenter = centerOfMass - boids[i].position;



[3] Correct implementation of the no neighbour wander rule
 - Completed this successfully
 - hasNeighbour flag that is calcualted from whether sepearation, alignment, or cohesion are not zero
 - If flag is false then wander rule is applied



[3] Correct implementation of obstacles rule
 - Completed this successfully
 - Used Physics.OverlapSphere(boids[i].position, obstacleCheckRadius)
 - for each collider the closest polint is found using obstacle.ClosestPoint(boids[i].position)
 - the normal is normal = boids[i].position - closestPoint
 - if normal is not zero then obstacleAvoidance += normal.normalized;




[3] Correct addition of the world boundary to the obstacles rule
 - Completed this successfully
 - Implemented in FixedUpdate() using a boundaryForce vector



[5] The total forces are accumulated correctly 
 - Completed this successfully
 - for every single boid total force has a zero vector
 - each rule contributes a term (weight * (ruleDirection * boidForceScale - boids[i].velocity);)
 - this matches the formula given (ωₖ ((ruleₖᵢ * α) − vᵢ))





[3] SetGoal is implemented correctly, implementing the behaviour described above. A target is set, path calculated.
 - Completed this successfully
 - Impleted in Sectgoal(Vector3 goal)


[5] boidzero is handled correctly. A check is made to ensure we should be processing the path (navigating + ready + enough corners). Position is correctly sampled from the navmesh. Bookkeeping for corners is correct and all corners are followed. Finishing the path is handled cleanly and variables are reset as needed.
 - Completed this successfully
 - path gollowing logic is applied when i = 0 in FixedUpdate()
 - path is proccessed when boidZeroNavigatingTowardGoal == true, boidZeroPath != null, boidZeroPath.status == NavMeshPathStatus.PathComplete, boidZeroPath.corners.Length > 1
 - Boid 0's NavMesh is checked against current corner
 - when currectCorner reaches the end then boidZeroPath.ClearCorners();, currentCorner = 0;, boidZeroNavigatingTowardGoal = false;



[2] The boid objects (animated meshes) are update correctly and follow the path and heading of the boid particles.
 - Completed this successfully
 - In the end of integration loop in FixedUpdate each boidOject has boidObjects[i].position = boids[i].position;
 - if boids[i].forward is not zero then boidObjects[i].rotation = Quaternion.LookRotation(boids[i].forward);


[2] The symplectic Euler integration scheme is implemented correctly.
 - Completed this successfully
 - Implemented at the end of FixedUpdate() in the second for loop.
 - velocity is updated first using the current total force boids[i].velocity += boids[i].currentTotalForce * dt;
 - Speed is clamped to maxSpeed
 - Position is updated using the new velocity boids[i].position += boids[i].velocity * dt;


[3] The simulator loop correctly updates all boid states using the correct update callback and time. Recall the lesson on simulator loops.
 - Completed this successfully
 - all simulation logic is inside FixedUpdate()
 - The time step dt is Time.fixedDeltaTime
  - loop processes all boids every FixedUpdate() frame


[1] The recorded testcase requires that the default values in the testcase are preserved.
 - Completed this successfully
 - All test cases preserved from the original file