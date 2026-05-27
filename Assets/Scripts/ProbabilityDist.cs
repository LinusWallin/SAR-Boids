using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Probability distribution of target location
/// </summary>
/// <author>Linus Wallin<author/>
public class ProbabilityDist : MonoBehaviour
{
    BoidSettings boidSettings;
    const int threadGroupSize = 1024;
    float kAtt;
    float kRep;
    int numCells;
    Vector3 gridStart;
    Vector3 gridSize;
    Vector3 cellSize;
    Vector3 targetPos;
    Vector3[] probGridVec;
    Vector3[] modifiedGridVec;
    Vector3[] startPos;
    Vector3[] pathData;
    float[] probGrid;
    int[] obstaclePos;
    int[] pathStepsData;
    List<Vector3>[] path;
    LayerMask obstacleMask;
    ComputeShader potentialCompute;
    
    // ── compute buffers
    ComputeBuffer obstacleBuffer;
    ComputeBuffer startBuffer;
    ComputeBuffer gridBuffer;
    ComputeBuffer pathStepsBuffer;
    ComputeBuffer pathBuffer;
    ComputeBuffer modifiedBuffer;
    ComputeBuffer posHistBuffer;
    ComputeBuffer virtualObsBuffer;
    ComputeBuffer agentMAPFBuffer;

    /// <summary>
    /// Initializes potential field
    /// </summary>
    /// <param name="obsMask">Obstacle layer int</param>
    /// <param name="start">Start position of the grid</param>
    /// <param name="gSize">Number of cells in each direction</param>
    /// <param name="cSize">Size of each cell</param>
    /// <param name="tPos">Target position</param>
    /// <param name="sPos">Start position of each boid</param>
    /// <param name="comp">Compute shader</param>
    /// <param name="settings">Settings for the simulation</param>
    public void Init(
            LayerMask obsMask,
            Vector3 start,
            Vector3 gSize,
            Vector3 cSize,
            Vector3 tPos,
            Vector3[] sPos,
            ComputeShader comp,
            BoidSettings settings
        )
    {
        obstacleMask = obsMask;
        gridStart = start;
        gridSize = gSize;
        cellSize = cSize;
        targetPos = tPos;
        startPos = sPos;
        potentialCompute = comp;
        boidSettings = settings;
        kAtt = boidSettings.kAtt;
        kRep = boidSettings.kRep;
        numCells = (int)(gridSize.x * gridSize.y * gridSize.z);
        probGridVec = new Vector3[numCells];
        modifiedGridVec = new Vector3[numCells];
        pathStepsData = new int[boidSettings.numBoids];
        pathData = new Vector3[boidSettings.numBoids * boidSettings.maxSteps];
        ProbabilityGrid();
    }

    /// <summary>
    /// Creates arrays to store grid information
    /// </summary>
    private void ProbabilityGrid()
    {
        probGrid = new float[numCells];
        Vector3 pos = new Vector3();
        List<int> obstacleList = new List<int>();
        for (int k = 0; k < gridSize.z; k++)
        {
            for (int j = 0; j < gridSize.y; j++)
            {
                for (int i = 0; i < gridSize.x; i++)
                {
                    pos.x = gridStart.x + i * cellSize.x;
                    pos.y = gridStart.y + j * cellSize.y;
                    pos.z = gridStart.z + k * cellSize.z;
                    bool isBoundary = i == 0 || i == gridSize.x - 1 ||
                                    j == 0 || j == gridSize.y - 1 ||
                                    k == 0 || k == gridSize.z - 1;
                    if (isBoundary)
                    {
                        int index = i + (int)(j * gridSize.x) + (int)(k * gridSize.x * gridSize.y);
                        obstacleList.Add(index);
                    }
                    else if(IsObstaclePosition(pos))
                    {
                        int index = i + (int)(j * gridSize.x) + (int)(k * gridSize.x * gridSize.y);
                        probGrid[index] = float.MaxValue;
                        obstacleList.Add(index);
                    }
                }
            }
        }
        obstaclePos = obstacleList.ToArray();
    }

    /// <summary>
    /// Calculates the potential field and generates a path for each boid
    /// </summary>
    /// <returns>Potential field</returns>
    public Vector3[] GetProbGrid()
    {
        int k = potentialCompute.FindKernel("CSProbabilityMain");
        int j = potentialCompute.FindKernel("CSPotentialPathMain");
        int totalCells = probGrid.Length;
        obstacleBuffer = new ComputeBuffer(obstaclePos.Length, sizeof(int));
        obstacleBuffer.SetData(obstaclePos);
        startBuffer = new ComputeBuffer(boidSettings.numBoids, sizeof(float) * 3);
        startBuffer.SetData(startPos);
        gridBuffer = new ComputeBuffer(totalCells, sizeof(float) * 3);
        pathStepsBuffer = new ComputeBuffer(boidSettings.numBoids, sizeof(int));
        pathBuffer = new ComputeBuffer(boidSettings.numBoids * boidSettings.maxSteps, sizeof(float) * 3);

        //Global compute shader parameters
        SetShaderVariables();

        //Buffers for compute shader APF calculation
        potentialCompute.SetBuffer(k, "obstaclePos", obstacleBuffer);
        potentialCompute.SetBuffer(k, "probGrid", gridBuffer);

        int threadGroups = Mathf.CeilToInt(totalCells / (float)threadGroupSize);
        potentialCompute.Dispatch(k, threadGroups, 1, 1);
        
        gridBuffer.GetData(probGridVec);

        if (boidSettings.isMAPF || boidSettings.isPath) {
            modifiedBuffer = new ComputeBuffer(totalCells, sizeof(float) * 3);
            modifiedBuffer.SetData(probGridVec);

            //Buffers for compute shader MAPF and path calculation
            potentialCompute.SetBuffer(j, "obstaclePos", obstacleBuffer);
            potentialCompute.SetBuffer(j, "startPosBuffer", startBuffer);
            potentialCompute.SetBuffer(j, "pathStepsBuffer", pathStepsBuffer);
            potentialCompute.SetBuffer(j, "probGrid", gridBuffer);
            potentialCompute.SetBuffer(j, "modifiedProbGrid", modifiedBuffer);
            potentialCompute.SetBuffer(j, "pathBuffer", pathBuffer);

            //Sets MAPF specific shader buffers
            //if (boidSettings.isMAPF)
            //{
            posHistBuffer = new ComputeBuffer(
                boidSettings.numBoids * boidSettings.histSize, sizeof(float) * 3
            );
            virtualObsBuffer = new ComputeBuffer(
                boidSettings.numBoids * boidSettings.maxVirtualObs, sizeof(float) * 3
            );

            potentialCompute.SetBuffer(j, "posHistoryBuffer", posHistBuffer);
            potentialCompute.SetBuffer(j, "virtualObsBuffer", virtualObsBuffer);

            var agentMAPFData = new AgentMAPFData[boidSettings.numBoids];
            for (int m = 0; m < boidSettings.numBoids; m++) {
                agentMAPFData[m].dynKAttr = boidSettings.kAtt;
                agentMAPFData[m].dynKRep = boidSettings.kRep;
            }
            agentMAPFBuffer = new ComputeBuffer(
                boidSettings.numBoids,
                AgentMAPFData.Size
            );
            agentMAPFBuffer.SetData(agentMAPFData);

            potentialCompute.SetBuffer(j, "data", agentMAPFBuffer);

            //}

            int agentGroups = Mathf.CeilToInt(boidSettings.numBoids / (float)64);
            potentialCompute.Dispatch(j, agentGroups, 1, 1);
            modifiedBuffer.GetData(modifiedGridVec);

            if (boidSettings.isPath) {
                pathStepsBuffer.GetData(pathStepsData);
                pathBuffer.GetData(pathData);
                path = GetPGDPathList();
            }
        }

        ReleaseBuffers();

        return probGridVec;
    }

    /// <summary>
    /// Sets variables for the compute shader
    /// </summary>
    void SetShaderVariables() {
        potentialCompute.SetBool("isMAPF", boidSettings.isMAPF);
        potentialCompute.SetInt("xMax", (int)gridSize.x);
        potentialCompute.SetInt("yMax", (int)gridSize.y);
        potentialCompute.SetInt("zMax", (int)gridSize.z);
        potentialCompute.SetInt("numObs", obstaclePos.Length);
        potentialCompute.SetInt("numAgents", boidSettings.numBoids);
        potentialCompute.SetInt("maxSteps", boidSettings.maxSteps);
        potentialCompute.SetInt("historySize", boidSettings.histSize);
        potentialCompute.SetInt("maxVirtualObs", boidSettings.maxVirtualObs);
        potentialCompute.SetFloat("revisitedDist", boidSettings.revisitedDist);
        potentialCompute.SetFloat("kRepMax", boidSettings.kRepMax * kRep);
        potentialCompute.SetFloat("kRepMin", boidSettings.kRepMin * kRep);
        potentialCompute.SetFloat("kAttrMax", boidSettings.kAttrMax * kAtt);
        potentialCompute.SetFloat("kAttrMin", boidSettings.kAttrMin * kAtt);
        potentialCompute.SetFloat("repChange", boidSettings.repKChange);
        potentialCompute.SetFloat("repNoChange", boidSettings.repKNoChange);
        potentialCompute.SetFloat("attChange", boidSettings.attKChange);
        potentialCompute.SetFloat("attNoChange", boidSettings.attKNoChange);
        potentialCompute.SetFloat("D", boidSettings.D);
        potentialCompute.SetFloat("dIO", boidSettings.obstacleInfluence);
        potentialCompute.SetFloat("kAttractive", kAtt);
        potentialCompute.SetFloat("kRepulsive", kRep);
        potentialCompute.SetFloat("minGradient", boidSettings.minGradient);
        potentialCompute.SetFloat("dt", boidSettings.pathStepSize);
        potentialCompute.SetFloat("minGoalDistance", boidSettings.goalRadius / 2.0f);
        potentialCompute.SetVector("cellSize", cellSize);
        potentialCompute.SetVector("gridStart", gridStart);
        potentialCompute.SetVector("qGoal", targetPos);
    }

    /// <summary>
    /// Releases all the buffers used for the potential field computation
    /// </summary>
    void ReleaseBuffers() {
        obstacleBuffer?.Release();
        startBuffer?.Release();
        gridBuffer?.Release();
        pathStepsBuffer?.Release();
        pathBuffer?.Release();
        modifiedBuffer?.Release();
        posHistBuffer?.Release();
        virtualObsBuffer?.Release();
        agentMAPFBuffer?.Release();
    }

    /// <summary>
    /// Reads path data and creates arrays for each boid of its path information
    /// </summary>
    /// <returns></returns>
    public List<Vector3>[] GetPGDPathList() {
        List<Vector3>[] paths = new List<Vector3>[boidSettings.numBoids];
        for (int i = 0; i < boidSettings.numBoids; i++) {
            int s = i * boidSettings.maxSteps;
            paths[i] = new List<Vector3>();
            for (int j = 0; j < pathStepsData[i]; j++) {
                paths[i].Add(pathData[s + j]);
            }
        }
        return paths;
    }

    /// <summary>
    /// Gets the path list containing information about each boids individual path
    /// </summary>
    /// <returns></returns>
    public List<Vector3>[] GetPGDPath() {
        return path;
    }

    /// <summary>
    /// Takes the list of paths and converts it into an index based array,
    /// which is readable in compute files
    /// </summary>
    /// <returns></returns>
    public Vector3[] GetPathArray() {
        Vector3[] pathArr = new Vector3[boidSettings.numBoids * boidSettings.maxSteps];
        for (int i = 0; i < boidSettings.numBoids; i++) {
            for (int j = 0; j < path[i].Count; j++) {
                pathArr[i * boidSettings.maxSteps + j] = path[i][j];
            }
        }
        return pathArr;
    }

    /// <summary>
    /// Gets the step count for each path as an array
    /// </summary>
    /// <returns></returns>
    public int[] GetPathStepsData() {
        return pathStepsData;
    }

    /// <summary>
    /// Checks if the position is inside an obstacle collider
    /// </summary>
    /// <param name="pos">The current inspected position</param>
    /// <returns></returns>
    private bool IsObstaclePosition(Vector3 pos) {
        return Physics.CheckBox(
            pos,
            cellSize * 0.5f,
            Quaternion.identity,
            obstacleMask
        );
    }

    public int[] GetObstaclePositions()
    {
        return obstaclePos;
    }

    public struct AgentMAPFData {
        public int histCount;
        public int virtObsCount;
        public int inRecentLm;
        public float dynKRep;
        public float dynKAttr;
        
        public static int Size
        {
            get
            {
                return sizeof(uint) * 3 + sizeof(float) * 2;
            }
        }
    }

}