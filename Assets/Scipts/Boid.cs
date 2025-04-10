using System;
using UnityEngine;

/// <summary>
/// Keeps track of and updates the position and velocity of the boid
/// </summary>
/// <author>Linus Wallin<author/>
public class Boid : MonoBehaviour
{
    BoidSettings boidSettings;

    public bool isAlive;
    public bool isLeader;
    public float speed;
    public int numFlockmates;
    public Vector3 direction;
    public Vector3 acceleration;
    public Vector3 position{
        get {
            return new Vector3(transform.position.x, transform.position.y, transform.position.z);
        }
    }
    public Vector3 separationForce;
    public Vector3 alignmentForce;
    public Vector3 cohesionForce;
    public Vector3 flockCenter;
    public GameObject target;
    

    private void Start()
    {
        
    }

    /// <summary>
    /// Initializes the boid
    /// </summary>
    /// <param name="boidSettings">Settings for the boids in the simulation</param>
    /// <param name="direction">Initial direction of the boid</param>
    /// <param name="speed">Initial speed of the boid</param>
    public void Init(BoidSettings boidSettings, Vector3 direction, float speed, bool lifeStatus) {
        this.boidSettings = boidSettings;
        this.direction = direction;
        this.speed = speed;
        this.isAlive = lifeStatus;
        this.numFlockmates = 0;
        this.flockCenter = new Vector3();

        transform.forward = direction;
        isLeader = false;
    }

    /// <summary>
    /// Updates the parameters of the boid
    /// </summary>
    public void UpdateBoid()
    {

        acceleration = new Vector3();

        CohesionRule();
        SeparationRule();
        AlignmentRule();
        
        if (isLeader) {
            Debug.DrawLine(transform.position, transform.position + cohesionForce, Color.red, 0.01f);
            Debug.DrawLine(transform.position, transform.position + separationForce, Color.blue, 0.01f);
            Debug.DrawLine(transform.position, transform.position + alignmentForce, Color.green, 0.1f);
            
            Debug.DrawLine(transform.position, target.transform.position, Color.yellow, 0.01f);
        }

        acceleration += cohesionForce;
        acceleration += separationForce;
        acceleration += alignmentForce;

        if (isLeader) {
            Debug.DrawLine(transform.position, transform.position + (acceleration * 4), Color.black, 0.01f);
            Debug.Log("Direction before: " + direction + ", Acceleration: " + acceleration);
        }
        

        direction += acceleration * Time.deltaTime;
        if (isLeader) {Debug.Log("Direction after: " + direction);}
        speed = direction.magnitude;
        direction /= speed;
        Debug.DrawLine(transform.position, transform.position + (direction * 4), Color.magenta, 0.01f);
        speed = Mathf.Clamp(speed, boidSettings.minSpeed, boidSettings.maxSpeed);
        direction *= speed;

        transform.Translate(direction * Time.deltaTime, Space.World);
        transform.forward = direction;
    }

    /// <summary>
    /// Applies the cohesion rule to the boid
    /// </summary>
    private void CohesionRule() {
        flockCenter /= numFlockmates;
        Vector3 cohesionDir = (flockCenter - position).normalized;
        cohesionForce = cohesionDir / boidSettings.cohesionWeight;
    }

    /// <summary>
    /// Applies the separtion rule to the boid
    /// </summary>
    private void SeparationRule() {
        separationForce /= boidSettings.separationWeight;
    }

    /// <summary>
    /// Applies the alignment rule to the boid
    /// </summary>
    private void AlignmentRule() {
        if (isLeader && target != null) {
            Vector3 compassDir = target.transform.position - position;
            alignmentForce += compassDir * 
            Mathf.Max(
                1, 
                boidSettings.numBoids * boidSettings.leaderInfluence
            );
            //Debug.DrawLine(transform.position, transform.position + alignmentForce, Color.red, 0.01f);
        }
        Vector3 normalizedAlignment = (
            alignmentForce/
            (boidSettings.numBoids - 1 + (isLeader ? 1 : 0))
        ).normalized;
        alignmentForce = normalizedAlignment / boidSettings.alignmentWeight;
        //Debug.DrawLine(transform.position, transform.position + alignmentForce, Color.blue, 0.01f);
    }

}
