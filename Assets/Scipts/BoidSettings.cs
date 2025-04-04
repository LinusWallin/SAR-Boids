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
    public float boidRadius;
    
    public float minSpeed;
    public float maxSpeed;
    public float maxSteerForce;
    
    public float separationWeight;
    public float alignmentWeight;
    public float cohesionWeight;
    
    public float neighborMaxDist;
    public float desiredDist;

}
