using UnityEngine;

/// <summary>
/// Settings for the boid search and rescue simulation
/// </summary>
/// <author>Linus Wallin<author/>
/// <version>1.0<version/>
[CreateAssetMenu(fileName = "BoidSettings", menuName = "Scriptable Objects/BoidSettings")]
public class BoidSettings : ScriptableObject
{
    public int numBoids;
    public int startCols;
    public int startRows;
    public int startDepth;
    public int sepRatio;
    public int leaders;
    public float leaderInfluence;
    public float boidRadius;
    public Vector3 startPosition;

    public float minSpeed;
    public float maxSpeed;
    public float maxSteerForce;

    public float separationWeight;
    public float alignmentWeight;
    public float cohesionWeight;
    public float OSQP_DS;
    public float OSQP_C;

    public float neighborMaxDist;
    public float desiredDist;
    public float startDist;

    public bool potentialField;
    public bool isCBF;
    public float cellRadius;
    public Vector3 gridSize;
    public float obstacleInfluence;
    public float kAtt;
    public float kRep;
}
