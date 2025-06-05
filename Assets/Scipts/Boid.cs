using System;
using UnityEngine;
using System.Collections.Generic;

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
    public List<Vector3> neighborPos;
    public Vector3 direction;
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
    
    
    public double osqpTime;

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

        CohesionRule();
        SeparationRule();
        AlignmentRule();

        Vector3 newDir = new Vector3();
        newDir += cohesionForce;
        newDir += separationForce;
        newDir += alignmentForce;
        direction = Vector3.RotateTowards(
            direction, 
            newDir, 
            boidSettings.maxSteerForce * Time.deltaTime, 
            0f
        );
        direction = direction.normalized;
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        Vector3 osqpDirection = OSQPSolver.RunOSQPSolver(this, boidSettings.OSQP_DS, boidSettings.OSQP_C);
        sw.Stop();
        osqpTime = sw.Elapsed.TotalMilliseconds;
        if (osqpDirection.magnitude < 1000) {
            direction = osqpDirection.normalized;
        }
        speed = Mathf.Clamp(speed, boidSettings.minSpeed, boidSettings.maxSpeed);
        transform.Translate(direction * speed * Time.deltaTime, Space.World);
        transform.forward = direction;
    }

    /// <summary>
    /// Applies the cohesion rule to the boid
    /// </summary>
    private void CohesionRule() {
        flockCenter /= numFlockmates == 0 ? 1 : numFlockmates;
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
                numFlockmates * boidSettings.leaderInfluence
            );
        }
        int totalFlock = numFlockmates + (isLeader ? 1 : 0);
        Vector3 normalizedAlignment = (
            alignmentForce/
            (totalFlock == 0 ? 1 : totalFlock)
        ).normalized;
        alignmentForce = normalizedAlignment / boidSettings.alignmentWeight;
    }

}
