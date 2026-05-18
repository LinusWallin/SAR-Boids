using UnityEngine;

/// <summary>
/// Settings for the boid search and rescue simulation
/// </summary>
/// <author>Linus Wallin<author/>
/// <version>1.0<version/>
[CreateAssetMenu(fileName = "BoidSettings", menuName = "Scriptable Objects/BoidSettings")]
public class BoidSettings : ScriptableObject
{
    [Header("Boid Settings")]
    public int numBoids;
    public int sepRatio;
    public int leaders;
    public float leaderInfluence;
    public float boidRadius;

    public float minSpeed;
    public float maxSpeed;
    public float maxSteerForce;

    public float separationWeight;
    public float alignmentWeight;
    public float cohesionWeight;
    public float neighborMaxDist;
    public float desiredDist;
    public float startDist;
    public float goalRadius;
    public float goalRadiusBuffer;

    [Header("Control Barrier Function Settings")]
    public float OSQP_DS;
    public float OSQP_C;
    public bool isCBF;

    [Header("APF Settings")]
    public bool potentialField;
    public float cellRadius;
    public Vector3 worldSize;
    public Vector3 gridSize;
    public float obstacleInfluence;
    
    public float kAtt;
    public float kRep;
    public float obstacleRadius;
    
    [Header("Path Following Settings")]
    public bool isPath;
    public float pathStepSize;
    public float kPath;
    public float kForward;
    public float minPathDist;
    public int maxSteps;
    
    [Header("MAPF Settings")]
    public bool isMAPF;
    public int histSize;
    public int maxVirtualObs;
    public float minGradient;
    public float D;
    public float revisitedDist;
    public float kRepMax;
    public float kRepMin;
    public float kAttrMax;
    public float kAttrMin;
    public float repKChange;
    public float repKNoChange;
    public float attKChange;
    public float attKNoChange;

    [Header("Visualization Options")]
    public bool showForcesOnBoid;
    public bool showPotField;
    public bool showGeneratedPath;
    public bool showGridObstacles;
    public bool showVisited;

    [Header("Evaluation")]
    public int saveInterval;
    public float timeLimit;
}
