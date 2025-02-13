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
    /// <param name="direction">Vector3: initial direction of the boid</param>
    /// <param name="speed">float: initial speed of the boid</param>
    public void Init(BoidSettings boidSettings, Vector3 direction, float speed) {

    }

    /// <summary>
    /// 
    /// </summary>
    public void UpdateBoid()
    {
        
    }
}
