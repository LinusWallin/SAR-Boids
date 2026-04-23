using UnityEngine;
using System.Collections.Generic;

public class Evaluation : MonoBehaviour {

    int _xMax;
    int _yMax;
    int _zMax;
    int totalCells;
    float sightRadius;
    int[] obstaclePos;
    uint[] result;
    Vector3 gridStart;
    Vector3 cellSize;
    List<Vector3> visitedPositions;
    ComputeShader shader;

    /// <summary>
    /// Init for evaluation
    /// </summary>
    /// <param name="eCompute">Compute shader for coverage calculation</param>
    /// <param name="positions">Visited positions</param>
    /// <param name="gStart">Grid start position</param>
    /// <param name="cSize">Cell size</param>
    /// <param name="sightR">Sight radius</param>
    /// <param name="obsPos">Obstacle positions</param>
    /// <param name="xMax">Maximum X dimension</param>
    /// <param name="yMax">Maximum Y dimension</param>
    /// <param name="zMax">Maximum Z dimension</param>
    public void Init(
        ComputeShader eCompute,
        List<Vector3> positions, 
        Vector3 gStart, 
        Vector3 cSize, 
        float sightR, 
        int[] obsPos, 
        int xMax, 
        int yMax, 
        int zMax
    ){
        shader = eCompute;
        visitedPositions = positions;
        gridStart = gStart;
        cellSize = cSize;
        sightRadius = sightR;
        obstaclePos = obsPos;
        _xMax = xMax;
        _yMax = yMax;
        _zMax = zMax;
        
        totalCells = _xMax * _yMax * _zMax;
        result = new uint[totalCells];
    }

    /// <summary>
    /// Calculate coverage percentage based on visited positions and obstacles 
    /// using a compute shader for efficient processing.
    /// </summary>
    /// <returns></returns>
    public float GetCoverage() {
        int totalPositions = visitedPositions.Count;
        

        var visitedBuffer = new ComputeBuffer(totalPositions, sizeof(float) * 3);
        visitedBuffer.SetData(visitedPositions, 0, 0, totalPositions);

        uint[] coverageArray = new uint[totalCells];
        foreach (int idx in obstaclePos) {
            if (idx >= 0 && idx < totalCells) {
                coverageArray[idx] = 2; // Mark obstacles with a distinct value
            }
        }

        var coverageGridBuffer = new ComputeBuffer(totalCells, sizeof(uint));
        coverageGridBuffer.SetData(coverageArray, 0, 0, totalCells);

        int kernel = shader.FindKernel("CSCoverageMain");
        
        shader.SetInt("numPositions", totalPositions);
        shader.SetInt("xMax", _xMax);
        shader.SetInt("yMax", _yMax);
        shader.SetInt("zMax", _zMax);
        shader.SetFloat("sightRadius", sightRadius);
        shader.SetVector("gridStart", gridStart);
        shader.SetVector("cellSize", cellSize);

        shader.SetBuffer(kernel, "coverageGrid", coverageGridBuffer);
        shader.SetBuffer(kernel, "visitedPositions", visitedBuffer);

        shader.Dispatch(kernel, Mathf.CeilToInt(totalPositions / 64f), 1, 1);

        coverageGridBuffer.GetData(result);
        coverageGridBuffer.Release();
        visitedBuffer.Release();

        int seen = 0;
        foreach (var v in result)
        {
            if (v == 1) seen++;
        }
        int totalNonObstacleCells = totalCells - obstaclePos.Length;
        return seen / (float)totalNonObstacleCells * 100f;
    }

    /// <summary>
    /// Returns a list of positions that are covered (seen) based on the result from the compute shader.
    /// </summary>
    /// <returns></returns>
    public List<Vector3> GetCoveredPositions()
    {
        List<Vector3> coveredPostions = new List<Vector3>();
        for (uint idx = 0; idx < result.Length; idx++) {
            if (result[idx] == 1)
            {
                coveredPostions.Add(Utils.indexToGridPos(idx, _xMax, _yMax, cellSize, gridStart));
            }
        }

        return coveredPostions;
    }

}