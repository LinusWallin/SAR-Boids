using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using System.Linq;

public class Evaluation : MonoBehaviour {

    int _numBoids;
    int _xMax;
    int _yMax;
    int _zMax;
    int totalCells;

    int collisions;
    int reachedTarget;

    float sightRadius;
    float averageTime;
    float fastestTime;

    int[] obstaclePos;
    uint[] result;
    Boid[] _boids;

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
        Boid[] boids,
        float sightR, 
        int[] obsPos, 
        int xMax, 
        int yMax, 
        int zMax,
        int numBoids
    ){
        shader = eCompute;
        visitedPositions = positions;
        gridStart = gStart;
        cellSize = cSize;
        _boids = boids;
        sightRadius = sightR;
        obstaclePos = obsPos;
        _xMax = xMax;
        _yMax = yMax;
        _zMax = zMax;
        _numBoids = numBoids;
        
        totalCells = _xMax * _yMax * _zMax;
        result = new uint[totalCells];

        RunComputations();
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

    private void RunComputations()
    {
        float totalTime = 0;
        int count = 0;
        for (int i = 0; i < _numBoids; i++)
        {
            if (_boids[i].isGoal)
            {
                totalTime += _boids[i].timeToReachTarget;
                count++;
                reachedTarget++;
                if (0 < _boids[i].timeToReachTarget && _boids[i].timeToReachTarget < fastestTime)
                {
                    fastestTime = _boids[i].timeToReachTarget;
                }
            }
            else if (!_boids[i].isAlive)
            {
                collisions++;
            }

        }
        averageTime = count > 0 ? totalTime / count : 0;
        fastestTime = fastestTime == float.MaxValue ? 0 : fastestTime;
    }

    /// <summary>
    /// Outputs evaluation metrics, including coverage, simulation time,
    /// and average time to reach the target, to the debug log.
    /// </summary>
    /// <param name="evaluation">The evaluation object containing the results</param>
    public void PrintEvaluationResults(bool isCBF, double simTime, double osqpTimeMs, List<float> minDistances, int osqpComputations)
    {
        float coverage = GetCoverage();
        string avgMinDistance = minDistances.Average().ToString();
        Debug.Log("Coverage: " + coverage + "%");
        Debug.Log("Simulation Time: " + (simTime) + "s");
        Debug.Log("Average Time to Reach Target: " + averageTime + "s");
        Debug.Log("Fastest Time to Reach Target: " + fastestTime + "s");
        Debug.Log("Average Minimum Distance: " + avgMinDistance);
        Debug.Log("Collision Count: " + collisions);
        Debug.Log("Reached Target Count: " + reachedTarget);
        if (isCBF)
        {
            Debug.Log($"OSQP took on average: {osqpTimeMs / osqpComputations}ms");
        }

        WriteResultsToFile(coverage, simTime, osqpTimeMs, osqpComputations, avgMinDistance);
    }

    private void WriteResultsToFile(float coverage, double simTime, double osqpTimeMs, int osqpComputations, string avgMinDistance)
    {
        string sceneName = SceneManager.GetActiveScene().name;
        string folderPath = Path.Combine(Application.persistentDataPath, "EvaluationResults");

        // Create the folder if it doesn't exist
        Directory.CreateDirectory(folderPath);

        int fileNumber = 1;
        string filePath;
        do
        {
            filePath = Path.Combine(folderPath, $"{sceneName}_{fileNumber}.txt");
            fileNumber++;
        } while (File.Exists(filePath));


        string content = $"Coverage: {coverage}%\n" +
                         $"Simulation Time: {simTime}s\n" +
                         $"Average Time to Reach Target: {averageTime}s\n" +
                         $"Fastest Time to Reach Target: {fastestTime}s\n" +
                         $"Average Minimum Distance: {avgMinDistance}\n" +
                         $"Collision Count: {collisions}\n" +
                         $"Reached Target Count: {reachedTarget}\n";
        if (osqpComputations > 0)
        {
            content += $"OSQP took on average: {osqpTimeMs / osqpComputations}ms\n";
        }
        
        File.WriteAllText(filePath, content);

        Debug.Log("Saved evaluation results to: " + filePath);
    }

}