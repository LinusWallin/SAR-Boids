using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Simulates the boids in the search and rescue scenario
/// </summary>
/// <author>Linus Wallin<author/>
public class Simulation : MonoBehaviour
{
    bool finished;
    int maxNeighbors;
    int totalMaxNeighbors;
    int framesSinceSavedPos = 0;
    int minDistCount = 0;
    int osqpComputations = 0;
    const int threadGroupSize = 1024;
    float startTime;
    float averageMinDist = 0;
    double osqpTimeMs = 0;
    public BoidSettings boidSettings;
    public GameObject boidPrefab;
    public GameObject[] obstacles;
    public Transform startPositions;
    public Transform boundingBox;
    public ComputeShader compute;
    public ComputeShader potentialCompute;
    public ComputeShader evaluationCompute;

    ComputeBuffer pathBuffer;

    [SerializeField] private GameObject target;
    [SerializeField] private LayerMask obstacleMask;
    ProbabilityDist probDist;
    Vector3[] potentialField;
    List<Vector3>[] paths;
    List<Vector3> visitedPositions;
    List<Vector3> coveredPositions;
    Boid[] boids;
    Boid[] aliveBoids;
    Boid[] boidCMs;
    Boid[] boidsAtTarget;
    Vector3 gridStart;
    Vector3 cellSize;

    void Start()
    {
        finished = false;
        visitedPositions = new List<Vector3>();
        coveredPositions = new List<Vector3>();
        //boidSettings.gridSize = boidSettings.worldSize / boidSettings.cellRadius * 2;
        gridStart = GetGridStart();
        
        SpawnBoids();

        probDist = gameObject.AddComponent<ProbabilityDist>();

        cellSize = new Vector3(
                boidSettings.cellRadius * 2,
                boidSettings.cellRadius * 2,
                boidSettings.cellRadius * 2
            );

        probDist.Init(
            obstacleMask,
            gridStart,
            boidSettings.gridSize,
            cellSize,
            target.transform.position,
            aliveBoids.Select(b => b.position).ToArray(),
            potentialCompute,
            boidSettings
        );
        potentialField = probDist.GetProbGrid();
        
        paths = probDist.GetPGDPath();

        List<Boid[]> boidList = new List<Boid[]>();
        int numGhosts = 0;

        //Loops through the planes of the bounding box and places ghosts on them
        foreach (Transform wallObj in boundingBox)
        {
            Vector3 wallCenter = wallObj.position;
            Vector3 wallSize = wallObj.gameObject.GetComponent<MeshCollider>().bounds.size;
            Vector3 wallRot = wallObj.rotation.eulerAngles;

            Boid[] planeCMs = CreateCardinalMarks(wallCenter, wallSize, wallRot);
            boidList.Add(planeCMs);
            numGhosts += planeCMs.Length;
        }

        boidCMs = new Boid[numGhosts];

        //merging the arrays of the different planes into one ghost boid array
        int arrIndex = 0;
        foreach (Boid[] boidArr in boidList)
        {
            for (int i = 0; i < boidArr.Length; i++)
            {
                boidCMs[arrIndex] = boidArr[i];
                arrIndex++;
            }
        }

        //Isolated Danger Marks
        List<Boid> boidIDMs = new List<Boid>();
        foreach (GameObject obs in obstacles)
        {
            boidIDMs = CreateIDMs(obs, boidIDMs);
        }
        Boid[] arrayIDM = boidIDMs.ToArray();
        int numIDMs = arrayIDM.Length;

        maxNeighbors = (boidSettings.numBoids - 1 + numGhosts + numIDMs / 2);
        totalMaxNeighbors = maxNeighbors * boidSettings.numBoids;
        boids = new Boid[boidSettings.numBoids + numGhosts + numIDMs];

        for (int i = 0; i < boidSettings.numBoids; i++)
        {
            boids[i] = aliveBoids[i];
        }

        for (int j = 0; j < numGhosts; j++)
        {
            boids[boidSettings.numBoids + j] = boidCMs[j];
        }

        for (int k = 0; k < numIDMs; k++)
        {
            boids[boidSettings.numBoids + numGhosts + k] = arrayIDM[k];
        }

        boidsAtTarget = new Boid[boidSettings.numBoids];

        startTime = Time.time;
    }

    /// <summary>
    /// Calculates the start position of the potential field
    /// </summary>
    /// <returns>Grid start position</returns>
    Vector3 GetGridStart()
    {
        return new Vector3(
            GameObject.Find("RightPlane").transform.position.x
            + boidSettings.cellRadius,
            GameObject.Find("BottomPlane").transform.position.y
            + boidSettings.cellRadius,
            GameObject.Find("BackPlane").transform.position.z
            + boidSettings.cellRadius
        );
    }

    /// <summary>
    /// Spawns the boids in the scene so that the simulation
    /// can take place
    /// </summary>
    void SpawnBoids()
    {
        aliveBoids = new Boid[boidSettings.numBoids];
        for (int i = 0; i < boidSettings.numBoids; i++)
        {
            GameObject b = Instantiate(boidPrefab, transform);
            b.transform.position = startPositions.GetChild(i).transform.position;
            aliveBoids[i] = b.GetComponent<Boid>();
            Vector3 direction = new Vector3(
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed),
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed),
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed)
            );
            float speed = Random.Range(boidSettings.minSpeed, boidSettings.maxSpeed);
            aliveBoids[i].Init(
                boidSettings,
                direction,
                speed,
                i * boidSettings.maxSteps,
                true
            );
            aliveBoids[i].timeToReachTarget = Mathf.Infinity;
        }
        int[] leaderIndices = RandomBoidSubset(aliveBoids.Length, boidSettings.leaders);
        foreach (int leaderIdx in leaderIndices)
        {
            Boid leaderBoid = aliveBoids[leaderIdx].GetComponent<Boid>();
            leaderBoid.isLeader = true;
            leaderBoid.target = target;
        }
    }

    /// <summary>
    /// Gets a random subset of boid indices
    /// </summary>
    /// <param name="arrLen">The number of boids in the array</param>
    /// <param name="subsetSize">How many indices to randomize</param>
    /// <returns></returns>
    private int[] RandomBoidSubset(int arrLen, int subsetSize)
    {
        HashSet<int> indices = new HashSet<int>();
        int[] subset = new int[subsetSize];

        while (indices.Count < subsetSize)
        {
            int idx = Random.Range(0, arrLen - 1);
            if (indices.Add(idx))
            {
                subset[indices.Count - 1] = idx;
            }
        }

        return subset;
    }

    /// <summary>
    /// Spawns ghost boids on a plane with a given center point, size and rotation
    /// </summary>
    /// <param name="center">Center point of the plane</param>
    /// <param name="size">Size of the plane</param>
    /// <param name="rotation">Rotation of the plane</param>
    /// <returns>returns the boid array of ghosts</returns>
    Boid[] CreateCardinalMarks(Vector3 center, Vector3 size, Vector3 rotation)
    {

        Vector3 ghostDir = new Vector3();
        Vector2 planeCenter = new Vector2();
        Vector2 planeSize = new Vector2();
        string staticAxis = "";

        //floor and roof
        if (rotation == Vector3.zero || rotation == new Vector3(0, 180, 180))
        {
            planeCenter = new Vector2(center.x, center.z);
            planeSize = new Vector2(size.x, size.z);
            staticAxis = "Y";
            if (rotation == Vector3.zero)
            {
                ghostDir = new Vector3(0, 1, 0);
            }
            else
            {
                ghostDir = new Vector3(0, -1, 0);
            }
        }
        //walls along Z-axis
        else if (rotation.x == 0 && rotation.y == 0)
        {
            planeCenter = new Vector2(center.z, center.y);
            planeSize = new Vector2(size.z, size.y);
            staticAxis = "X";
            if (rotation.z == 270)
            {
                ghostDir = new Vector3(1, 0, 0);
            }
            else
            {
                ghostDir = new Vector3(-1, 0, 0);
            }
        }
        //walls along X-axis
        else if (rotation.y == 0 && rotation.z == 0)
        {
            planeCenter = new Vector2(center.x, center.y);
            planeSize = new Vector2(size.x, size.y);
            staticAxis = "Z";
            if (rotation.x == 90)
            {
                ghostDir = new Vector3(0, 0, 1);
            }
            else
            {
                ghostDir = new Vector3(0, 0, -1);
            }
        }

        int boidCols = (int)Mathf.Floor(planeSize.x / (boidSettings.sepRatio * boidSettings.boidRadius));
        int boidRows = (int)Mathf.Floor(planeSize.y / (boidSettings.sepRatio * boidSettings.boidRadius));
        int numGhostBoids = boidRows * boidCols;

        Vector2 botLeft = new Vector2(
            planeCenter.x - planeSize.x / 2,
            planeCenter.y - planeSize.y / 2
        );

        Vector2 cellSize = new Vector2(
            planeSize.x / boidCols,
            planeSize.y / boidRows
        );

        Boid[] boidCM = new Boid[numGhostBoids];
        Vector3 boidCMPos = new Vector3();

        for (int row = 0; row < boidRows; row++)
        {
            for (int col = 0; col < boidCols; col++)
            {
                if (staticAxis == "X")
                {
                    boidCMPos = new Vector3(
                        center.x,
                        botLeft.y + row * cellSize.y + cellSize.y / 2,
                        botLeft.x + col * cellSize.x + cellSize.x / 2
                    );
                }
                else if (staticAxis == "Y")
                {
                    boidCMPos = new Vector3(
                        botLeft.x + col * cellSize.x + cellSize.x / 2,
                        center.y,
                        botLeft.y + row * cellSize.y + cellSize.y / 2
                    );
                }
                else
                {
                    boidCMPos = new Vector3(
                        botLeft.x + col * cellSize.x + cellSize.x / 2,
                        botLeft.y + row * cellSize.y + cellSize.y / 2,
                        center.z
                    );
                }
                int ghostIdx = col + col * row;
                GameObject ghostBoid = Instantiate(boidPrefab, transform);
                ghostBoid.transform.position = boidCMPos;
                boidCM[ghostIdx] = ghostBoid.GetComponent<Boid>();
                boidCM[ghostIdx].Init(
                    boidSettings,
                    ghostDir,
                    0,
                    0,
                    false
                );
            }
        }
        return boidCM;
    }

    /// <summary>
    /// Places ghost boids in a grid formation on the faces
    /// of an obstacle.
    /// </summary>
    /// <param name="obstacle">The obstacle which the ghost boids should be placed on</param>
    /// <param name="boidIDMs">IDMs List to keep track of ghost boids</param>
    /// <returns>Returns updated IDM List</returns>
    List<Boid> CreateIDMs(GameObject obstacle, List<Boid> boidIDMs)
    {
        MeshCollider meshCollider = obstacle.GetComponent<MeshCollider>();
        Bounds bounds = meshCollider.sharedMesh.bounds; //Gets local bounds of obstacle
        Vector3 center = bounds.center;
        Vector3 extents = bounds.extents;
        Vector3 gridExtents = Vector3.Scale(extents, obstacle.transform.lossyScale);
        Vector3 boidsInDir = (gridExtents * 2) / (boidSettings.boidRadius * boidSettings.sepRatio);
        boidsInDir = new Vector3(
            Mathf.Floor(boidsInDir.x),
            Mathf.Floor(boidsInDir.y),
            Mathf.Floor(boidsInDir.z)
        );

        // Faces of game object
        // left, right, top, bottom, front, back
        // center, normal, row direction, column direction
        FaceData[] faces = new FaceData[] {
            new FaceData(
                obstacle,
                center + new Vector3(extents.x, 0, 0),
                Vector3.right,
                Vector3.forward,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center - new Vector3(extents.x, 0, 0),
                Vector3.left,
                Vector3.back,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center + new Vector3(0, extents.y, 0),
                Vector3.up,
                Vector3.right,
                Vector3.back
            ),
            new FaceData(
                obstacle,
                center - new Vector3(0, extents.y, 0),
                Vector3.down,
                Vector3.left,
                Vector3.back
            ),
            new FaceData(
                obstacle,
                center + new Vector3(0, 0, extents.z),
                Vector3.forward,
                Vector3.right,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center - new Vector3(0, 0, extents.z),
                Vector3.back,
                Vector3.left,
                Vector3.up
            )
        };

        foreach (FaceData face in faces)
        {
            Vector3 faceCenter = obstacle.transform.InverseTransformPoint(face.center);
            Vector3 normal = obstacle.transform.InverseTransformDirection(face.normal);
            Vector3 rowDir = obstacle.transform.InverseTransformDirection(face.rowDir);
            Vector3 colDir = obstacle.transform.InverseTransformDirection(face.colDir);

            Vector3 rowStepSize = Utils.Vec3Div(rowDir, obstacle.transform.lossyScale) * (boidSettings.boidRadius * boidSettings.sepRatio);
            Vector3 colStepSize = Utils.Vec3Div(colDir, obstacle.transform.lossyScale) * (boidSettings.boidRadius * boidSettings.sepRatio);
            Vector3 rowExt = Vector3.Scale(rowDir, extents);
            Vector3 colExt = Vector3.Scale(colDir, extents);
            Vector3 botLeft = faceCenter - rowExt - colExt;

            int numRows = Mathf.Abs((int)Vector3.Dot(rowDir, boidsInDir));
            int numCols = Mathf.Abs((int)Vector3.Dot(colDir, boidsInDir));

            Vector3 gridStart = botLeft;

            if (numRows == 0)
            {
                gridStart += rowExt;
                numRows = 1;
            }
            else
            {
                gridStart += rowStepSize / 2;
            }
            if (numCols == 0)
            {
                gridStart += colExt;
                numCols = 1;
            }
            else
            {
                gridStart += colStepSize / 2;
            }

            for (int x = 0; x < numRows; x++)
            {
                for (int y = 0; y < numCols; y++)
                {
                    Vector3 posIDM = gridStart + rowStepSize * x + colStepSize * y;
                    GameObject ghostBoid = Instantiate(boidPrefab, transform);
                    RaycastHit hit;
                    Vector3 surfaceNormal = normal;
                    Vector3 globalPos = obstacle.transform.TransformPoint(posIDM);
                    Vector3 globalNormal = obstacle.transform.TransformDirection(normal);
                    if (Physics.Raycast(globalPos + globalNormal, -globalNormal, out hit, obstacleMask))
                    {
                        surfaceNormal = hit.normal;
                        posIDM = hit.point;
                    }

                    ghostBoid.transform.position = posIDM;
                    Vector3 boidScale = boidPrefab.transform.localScale;
                    float maxScale = Mathf.Max(
                        Mathf.Max(
                            obstacle.transform.localScale.x,
                            obstacle.transform.localScale.y
                        ),
                        obstacle.transform.localScale.z
                    );
                    boidIDMs.Add(ghostBoid.GetComponent<Boid>());
                    boidIDMs[boidIDMs.Count - 1].Init(
                        boidSettings,
                        surfaceNormal,
                        0,
                        0,
                        false
                    );
                }
            }
        }
        return boidIDMs;
    }

    /// <summary>
    /// Debugging function that draws the lines of the potential field
    /// and the paths that are generated
    /// </summary>
    void OnDrawGizmos()
    {
        if (potentialField == null) return;
        if (potentialField.Length == 0) return;

        if (boidSettings.isPath && boidSettings.showGeneratedPath) {
            List<Vector3>[] paths = probDist.GetPGDPath();
            int[] pathSteps = probDist.GetPathStepsData();
            for (int i = 0; i < boidSettings.numBoids; i++) {
                int s  = pathSteps[i];
                for (int j = 1; j < s; j++) {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(paths[i][j-1], paths[i][j]);
                }
            }
        
        }

        if (boidSettings.showPotField)
        {
            for (int i = 0; i < boidSettings.gridSize.x; i++)
            {
                for (int j = 0; j < boidSettings.gridSize.y; j++)
                {
                    for (int k = 0; k < boidSettings.gridSize.z; k++)
                    {
                        if (i % 3 == 0 && j % 3 == 0 && k % 3 == 0)
                        {
                            int index = i + j * (int)boidSettings.gridSize.x + k * (int)(boidSettings.gridSize.x * boidSettings.gridSize.y);
                            Vector3 pos = gridStart + new Vector3(i, j, k) * boidSettings.cellRadius * 2;
                            Vector3 force = potentialField[index];

                            if (!float.IsNaN(force.x) && force != Vector3.zero)
                            {
                                Gizmos.color = Color.blue;
                                Gizmos.DrawLine(pos, pos + force.normalized * 0.4f * boidSettings.cellRadius);
                                Gizmos.color = Color.Lerp(Color.green, Color.red, force.magnitude / (100f * boidSettings.kAtt));
                                Gizmos.DrawLine(pos + force.normalized * 0.4f * boidSettings.cellRadius, pos + force.normalized * 5 * boidSettings.cellRadius);
                            }
                        }
                    }
                }
            }
        }

        if (boidSettings.showGridObstacles)
        {
            int[] obs = probDist.GetObstaclePositions();
            foreach(int index in obs)
            {
                Vector3 obsPos = Utils.indexToGridPos(
                    (uint)index, 
                    (int)boidSettings.gridSize.x, 
                    (int)boidSettings.gridSize.y, 
                    cellSize, 
                    gridStart
                );

                Gizmos.DrawWireCube(obsPos, cellSize);
            }
        }

        if (boidSettings.showVisited)
        {
            foreach (Vector3 pos in coveredPositions)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireCube(pos, cellSize);
            }
        }

    }

    /// <summary>
    /// Runs every frame and updates the positions of the boids 
    /// and the forces applied to them
    /// </summary>
    void Update()
    {
        if (finished) return;
        if (boids != null)
        {
            if (AllReachedTarget() || Time.time - startTime > boidSettings.timeLimit || AllDeadBoids())
            {
                Evaluation evaluation = gameObject.AddComponent<Evaluation>();
                evaluation.Init(
                    evaluationCompute,
                    visitedPositions,
                    gridStart,
                    cellSize,
                    boids,
                    boidSettings.neighborMaxDist,
                    probDist.GetObstaclePositions(),
                    (int)boidSettings.gridSize.x,
                    (int)boidSettings.gridSize.y,
                    (int)boidSettings.gridSize.z,
                    boidSettings.numBoids
                );

                PrintEvaluationResults(evaluation);

                if (boidSettings.showVisited)
                {
                    coveredPositions = evaluation.GetCoveredPositions();
                }

                finished = true;
            }
            else
            {
                bool savePos = false;
                var boidData = new BoidData[boids.Length];

                if (framesSinceSavedPos == boidSettings.saveInterval)
                {
                    savePos = true;
                    framesSinceSavedPos = 0;
                } else
                {
                    framesSinceSavedPos += 1;
                }

                for (int i = 0; i < boids.Length; i++)
                {
                    if (boids[i] != null)
                    {
                        boidData[i].position = boids[i].position;
                        boidData[i].direction = boids[i].direction;
                        if (boidSettings.isPath && i < boidSettings.numBoids) {
                            boidData[i].currentPathIndex = boids[i].pathIndex;
                            boidData[i].pathEndIndex = boidData[i].currentPathIndex + paths[i].Count;
                        }
                        if (boids[i].isAlive)
                        {
                            boidData[i].minNeighborDist = Mathf.Infinity;
                            boidData[i].isAlive = 1;
                            if (savePos)
                            {
                                visitedPositions.Add(boids[i].position);
                            }
                        }
                        else
                        {
                            boidData[i].isAlive = 0;
                        }
                        if (boids[i].isGoal)
                        {
                            boidData[i].goalReached = 1;
                        }
                        else
                        {
                            boidData[i].goalReached = 0;
                        }
                    }
                }

                var boidBuffer = new ComputeBuffer(boids.Length, BoidData.Size);
                boidBuffer.SetData(boidData);

                var neighborData = new NeighborData[totalMaxNeighbors];
                var neighborBuffer = new ComputeBuffer(
                    totalMaxNeighbors,
                    NeighborData.Size
                );
                neighborBuffer.SetData(neighborData);

                var fieldBuffer = new ComputeBuffer(potentialField.Length, sizeof(float) * 3);
                fieldBuffer.SetData(potentialField);

                
                //Set compute shader variables
                compute.SetBuffer(0, "boids", boidBuffer);
                compute.SetBuffer(0, "neighbors", neighborBuffer);
                compute.SetBuffer(0, "potentialField", fieldBuffer);

                if (boidSettings.isPath) {
                    Vector3[] pathArray = probDist.GetPathArray();
                    pathBuffer = new ComputeBuffer(pathArray.Length, sizeof(float) * 3);
                    pathBuffer.SetData(pathArray);
                    compute.SetBuffer(0, "path", pathBuffer);
                }
                // Sets a empty buffer if not using pathfinding to avoid errors in the compute shader
                else
                {
                    pathBuffer = new ComputeBuffer(1, sizeof(float) * 3);
                    pathBuffer.SetData(new Vector3[1]);
                    compute.SetBuffer(0, "path", pathBuffer);
                }
                
                compute.SetInt("numBoids", boids.Length);
                compute.SetInt("numAliveBoids", boidSettings.numBoids);
                compute.SetInt("maxNeighbors", maxNeighbors);
                compute.SetFloat("neighborMaxDist", boidSettings.neighborMaxDist);
                compute.SetFloat("desiredDist", boidSettings.desiredDist);
                compute.SetFloat("collisionDist", boidSettings.boidRadius * 2);
                compute.SetFloat("goalRadius", boidSettings.goalRadius);
                compute.SetFloat("kPath", boidSettings.kPath);
                compute.SetFloat("minPathDist", boidSettings.minPathDist);
                compute.SetBool("isPath", boidSettings.isPath);
                compute.SetBool("isField", boidSettings.potentialField);
                compute.SetFloat("kForward", boidSettings.kForward);
                compute.SetInts(
                    "gridSize",
                    (int)boidSettings.gridSize.x,
                    (int)boidSettings.gridSize.y,
                    (int)boidSettings.gridSize.z
                );
                compute.SetFloats(
                    "cellSize",
                    boidSettings.cellRadius * 2,
                    boidSettings.cellRadius * 2,
                    boidSettings.cellRadius * 2
                );
                compute.SetFloats(
                    "gridStart",
                    gridStart.x,
                    gridStart.y,
                    gridStart.z
                );
                compute.SetFloats(
                    "targetPos",
                    target.transform.position.x,
                    target.transform.position.y,
                    target.transform.position.z
                );
                compute.SetVector("boundaryMin", Utils.Vec3Mult(boidSettings.worldSize, new Vector3(-0.5f, -0.5f, -0.5f)));
                compute.SetVector("boundaryMax", Utils.Vec3Mult(boidSettings.worldSize, new Vector3(0.5f, 0.5f, 0.5f)));

                int threadGroups = Mathf.CeilToInt(boidSettings.numBoids / (float)threadGroupSize);
                compute.Dispatch(0, threadGroups, 1, 1);

                boidBuffer.GetData(boidData);
                neighborBuffer.GetData(neighborData);

                float averageNeighborDist = 0;
                int neighborDistCount = 0;

                for (int i = 0; i < boidSettings.numBoids; i++)
                {
                    if (boids[i] != null)
                    {
                        boids[i].isAlive = boidData[i].isAlive == 1;
                        if (boids[i].isAlive)
                        {
                            if (boidData[i].goalReached == 1)
                            {
                                boids[i].isGoal = true;
                                boids[i].timeToReachTarget = Time.time - startTime;
                                boids[i].isAlive = false;
                                boids[i].speed = 0;
                                boidsAtTarget[i] = boids[i];
                            }
                            else
                            {
                                boids[i].flockCenter = boidData[i].flockCenter;
                                boids[i].numFlockmates = boidData[i].numFlockmates;
                                boids[i].alignmentForce = boidData[i].flockDirection;
                                boids[i].separationForce = boidData[i].separationDirection.normalized;
                                if (boidSettings.isPath) {
                                    boids[i].pathIndex = boidData[i].currentPathIndex;
                                    boids[i].pathForce = boidData[i].pathForce;
                                    boids[i].forwardForce = boidData[i].forwardForce;
                                }
                                boids[i].neighborPos.Clear();

                                int startIdx = i * maxNeighbors;
                                for (int j = 0; j < boids[i].numFlockmates; j++)
                                {
                                    boids[i].neighborPos.Add(neighborData[startIdx + j].position);
                                }

                                boids[i].UpdateBoid();
                                osqpTimeMs += boids[i].osqpTime;
                                osqpComputations++;
                                averageNeighborDist += boidData[i].minNeighborDist;
                                neighborDistCount++;
                            }
                        }
                    }
                }

                if (neighborDistCount > 0)
                {
                    minDistCount++;
                    averageMinDist += averageNeighborDist / neighborDistCount;
                }

                //release the compute shader buffers
                boidBuffer.Release();
                neighborBuffer.Release();
                fieldBuffer.Release();
                pathBuffer.Release();
            }
            
        }
    }

    /// <summary>
    /// Outputs evaluation metrics, including coverage, simulation time,
    /// and average time to reach the target, to the debug log.
    /// </summary>
    /// <param name="evaluation">The evaluation object containing the results</param>
    private void PrintEvaluationResults(Evaluation evaluation) {
        float coverage = evaluation.GetCoverage();
        Debug.Log("Coverage: " + coverage + "%");
        Debug.Log("Simulation Time: " + (Time.time - startTime) + "s");
        Debug.Log("Average Time to Reach Target: " + evaluation.GetAverageTime() + "s");
        Debug.Log("Fastest Time to Reach Target: " + evaluation.GetFastestTime() + "s");
        Debug.Log("Average Minimum Distance: " + (minDistCount > 0 ? averageMinDist / minDistCount : "N/A"));
        Debug.Log("Collision Count: " + evaluation.GetCollisionCount());
        Debug.Log("Reached Target Count: " + evaluation.GetReachedTargetCount());
        if (boidSettings.isCBF)
        {
            Debug.Log($"OSQP took on average: {osqpTimeMs/osqpComputations}ms");
        }
    }

    /// <summary>
    /// Checks if all boids have reached the target
    /// </summary>
    /// <returns></returns>
    private bool AllReachedTarget()
    {
        for (int b = 0; b < boidSettings.numBoids; b++)
        {
            if (boidsAtTarget[b] == null)
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Determines whether all boids in the collection are null or not alive.
    /// </summary>
    /// <remarks>Iterates the boid collection and returns as soon as a live boid is found.</remarks>
    /// <returns>true if no boid is alive; otherwise, false.</returns>
    private bool AllDeadBoids()
    {
        for (int a = 0; a < boidSettings.numBoids; a++)
        {
            if (boids[a] != null && boids[a].isAlive)
            {
                return false;
            }
        }
        return true;
    }

    public struct FaceData
    {
        public GameObject obstacle;
        public Vector3 center;
        public Vector3 normal;
        public Vector3 rowDir;
        public Vector3 colDir;

        public FaceData(GameObject obstacle, Vector3 center, Vector3 normal, Vector3 rowDir, Vector3 colDir)
        {
            this.obstacle = obstacle;
            this.center = obstacle.transform.TransformPoint(center);
            this.normal = obstacle.transform.TransformDirection(normal);
            this.rowDir = obstacle.transform.TransformDirection(rowDir);
            this.colDir = obstacle.transform.TransformDirection(colDir);
        }
    }

    public struct BoidData
    {
        public Vector3 position;
        public Vector3 direction;

        public Vector3 flockDirection;
        public Vector3 flockCenter;
        public Vector3 separationDirection;
        public Vector3 pathForce;
        public Vector3 forwardForce;
        public float minNeighborDist;
        public int currentPathIndex;
        public int pathEndIndex;
        public int numFlockmates;
        public int isAlive;
        public int goalReached;

        public static int Size
        {
            get
            {
                return sizeof(float) * 3 * 7 + sizeof(int) * 5 + sizeof(float);
            }
        }
    }

    public struct NeighborData
    {
        public uint boidIdx;
        public Vector3 position;

        public static int Size
        {
            get
            {
                return sizeof(uint) + sizeof(float) * 3;
            }
        }
    }

}