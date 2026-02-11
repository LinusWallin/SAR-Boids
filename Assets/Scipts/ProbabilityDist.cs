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
    Vector3[] startPos;
    Vector3[] pathData;
    float[] probGrid;
    int[] obstaclePos;
    int[] pathStepsData;
    List<Vector3>[] path;
    LayerMask obstacleMask;
    ComputeShader potentialCompute;

    /// <summary>
    /// Initializes potential field
    /// </summary>
    /// <param name="obsMask">Obstacle layer int</param>
    /// <param name="start">Start position of the grid</param>
    /// <param name="gSize">Number of cells in each direction</param>
    /// <param name="cSize">Size of each cell</param>
    /// <param name="tPos">Target position</param>
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

                    Collider[] obstacles = Physics.OverlapBox(
                        pos,
                        cellSize / 2,
                        Quaternion.identity,
                        obstacleMask
                    );
                    if (obstacles.Length > 0)
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
    /// Calculates the potential field
    /// </summary>
    /// <returns>Potential field</returns>
    public Vector3[] GetProbGrid()
    {
        int k = potentialCompute.FindKernel("CSProbabilityMain");
        int j = potentialCompute.FindKernel("CSPotentialPathMain");
        int totalCells = probGrid.Length;
        var obstacleBuffer = new ComputeBuffer(obstaclePos.Length, sizeof(int));
        obstacleBuffer.SetData(obstaclePos);
        var startBuffer = new ComputeBuffer(boidSettings.numBoids, sizeof(float) * 3);
        startBuffer.SetData(startPos);
        var gridBuffer = new ComputeBuffer(totalCells, sizeof(float) * 3);
        var pathStepsBuffer = new ComputeBuffer(boidSettings.numBoids, sizeof(int));
        var pathBuffer = new ComputeBuffer(boidSettings.numBoids * boidSettings.maxSteps, sizeof(float) * 3);


        //Global compute shader parameters
        potentialCompute.SetBool("isMAPF", boidSettings.isMAPF);
        potentialCompute.SetInt("xMax", (int)gridSize.x);
        potentialCompute.SetInt("yMax", (int)gridSize.y);
        potentialCompute.SetInt("zMax", (int)gridSize.z);
        potentialCompute.SetInt("numObs", obstaclePos.Length);
        potentialCompute.SetInt("numAgents", boidSettings.numBoids);
        potentialCompute.SetInt("maxSteps", boidSettings.maxSteps);
        potentialCompute.SetFloat("D", boidSettings.D);
        potentialCompute.SetFloat("dIO", boidSettings.obstacleInfluence);
        potentialCompute.SetFloat("kAttractive", kAtt);
        potentialCompute.SetFloat("kRepulsive", kRep);
        potentialCompute.SetFloat("minGradient", boidSettings.minGradient);
        potentialCompute.SetFloat("cellStepSize", 2*boidSettings.cellRadius);
        potentialCompute.SetFloat("minGoalDistance", boidSettings.goalRadius);
        potentialCompute.SetVector("cellSize", cellSize);
        potentialCompute.SetVector("gridStart", gridStart);
        potentialCompute.SetVector("qGoal", targetPos);

        //Buffers for compute shader APF calculation
        potentialCompute.SetBuffer(k, "obstaclePos", obstacleBuffer);
        potentialCompute.SetBuffer(k, "probGrid", gridBuffer);

        int threadGroups = Mathf.CeilToInt(totalCells / (float)threadGroupSize);
        potentialCompute.Dispatch(k, threadGroups, 1, 1);
        
        gridBuffer.GetData(probGridVec);

        if (boidSettings.isMAPF) {
            var modifiedBuffer = new ComputeBuffer(totalCells, sizeof(float) * 3);
            modifiedBuffer.SetData(probGridVec);

            //Buffers for compute shader MAPF and path calculation
            potentialCompute.SetBuffer(j, "obstaclePos", obstacleBuffer);
            potentialCompute.SetBuffer(j, "startPosBuffer", startBuffer);
            potentialCompute.SetBuffer(j, "pathStepsBuffer", pathStepsBuffer);
            potentialCompute.SetBuffer(j, "probGrid", modifiedBuffer);
            potentialCompute.SetBuffer(j, "pathBuffer", pathBuffer);

            int agentGroups = Mathf.CeilToInt(boidSettings.numBoids / (float)64);
            potentialCompute.Dispatch(j, agentGroups, 1, 1);
            modifiedBuffer.GetData(probGridVec);

            if (boidSettings.isPath) {
                pathStepsBuffer.GetData(pathStepsData);
                pathBuffer.GetData(pathData);
                path = GetPGDPathList();
            }
        }

        obstacleBuffer.Release();
        gridBuffer.Release();

        startBuffer.Release();
        pathStepsBuffer.Release();
        pathBuffer.Release();

        return probGridVec;
    }

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

    public List<Vector3>[] GetPGDPath() {
        return path;
    }

    public Vector3[] GetPathArray() {
        Vector3[] pathArr = new Vector3[boidSettings.numBoids * boidSettings.maxSteps];
        for (int i = 0; i < boidSettings.numBoids; i++) {
            for (int j = 0; j < path[i].Count; j++) {
                pathArr[i * boidSettings.maxSteps + j] = path[i][j];
            }
        }
        return pathArr;
    }

    public int[] GetPathStepsData() {
        return pathStepsData;
    }

}