using UnityEngine;

[CreateAssetMenu(fileName = "BoidSettings", menuName = "Scriptable Objects/BoidSettings")]
public class BoidSettings : ScriptableObject
{
    public int numBoids;
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
