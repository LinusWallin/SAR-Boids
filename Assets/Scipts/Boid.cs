using UnityEngine;

/// <summary>
/// Keeps track of and updates the position and velocity of the boid
/// </summary>
/// <author>Linus Wallin<author/>
/// <version>1.0<version/>
public class Boid : MonoBehaviour
{
    BoidSettings boidSettings;

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
    

    private void Start()
    {
        
    }

    /// <summary>
    /// Initializes the boid
    /// </summary>
    /// <param name="boidSettings">Settings for the boids in the simulation</param>
    /// <param name="direction">Initial direction of the boid</param>
    /// <param name="speed">Initial speed of the boid</param>
    public void Init(BoidSettings boidSettings, Vector3 direction, float speed) {
        this.boidSettings = boidSettings;
        this.direction = direction;
        this.speed = speed;
        this.acceleration = new Vector3();
        this.numFlockmates = 0;
        this.flockCenter = new Vector3();
    }

    /// <summary>
    /// Updates the parameters of the boid
    /// </summary>
    public void UpdateBoid()
    {
        CohesionRule();
        SeparationRule();
        AlignmentRule();

        acceleration += cohesionForce;
        acceleration += separationForce;
        acceleration += alignmentForce;

        direction += acceleration * Time.deltaTime;
        speed = direction.magnitude;
        direction /= speed;
        speed = Mathf.Clamp(speed, boidSettings.minSpeed, boidSettings.maxSpeed);
        direction *= speed;

        transform.Translate(direction * Time.deltaTime, Space.World);
        transform.right = direction;
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
        alignmentForce /= boidSettings.alignmentWeight;
    }

}
