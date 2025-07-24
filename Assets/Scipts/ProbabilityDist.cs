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
    float[] probGrid;
    int[] obstaclePos;
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
            ComputeShader comp,
            BoidSettings settings
        )
    {
        obstacleMask = obsMask;
        gridStart = start;
        gridSize = gSize;
        cellSize = cSize;
        targetPos = tPos;
        potentialCompute = comp;
        boidSettings = settings;
        kAtt = boidSettings.kAtt;
        kRep = boidSettings.kRep;
        numCells = (int)(gridSize.x * gridSize.y * gridSize.z);
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
        int totalCells = probGrid.Length;
        var obstacleBuffer = new ComputeBuffer(obstaclePos.Length, sizeof(int));
        obstacleBuffer.SetData(obstaclePos);
        var gridBuffer = new ComputeBuffer(totalCells, sizeof(float) * 3);

        potentialCompute.SetInt("xMax", (int)gridSize.x);
        potentialCompute.SetInt("yMax", (int)gridSize.y);
        potentialCompute.SetInt("zMax", (int)gridSize.z);
        potentialCompute.SetInt("numObs", obstaclePos.Length);
        potentialCompute.SetFloat("dIO", boidSettings.obstacleInfluence);
        potentialCompute.SetFloat("kAttractive", kAtt);
        potentialCompute.SetFloat("kRepulsive", kRep);
        potentialCompute.SetVector("cellSize", cellSize);
        potentialCompute.SetVector("gridStart", gridStart);
        potentialCompute.SetVector("qGoal", targetPos);

        potentialCompute.SetBuffer(k, "obstaclePos", obstacleBuffer);
        potentialCompute.SetBuffer(k, "probGrid", gridBuffer);

        int threadGroups = Mathf.CeilToInt(totalCells / (float)threadGroupSize);
        potentialCompute.Dispatch(k, threadGroups, 1, 1);

        Vector3[] newProbGrid = new Vector3[totalCells];
        gridBuffer.GetData(newProbGrid);

        obstacleBuffer.Release();
        gridBuffer.Release();

        return newProbGrid;
    }

}