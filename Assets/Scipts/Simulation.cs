using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simulates the boids in the search and rescue scenario
/// </summary>
/// <author>Linus Wallin<author/>
/// <version>1.0<version/>
public class Simulation : MonoBehaviour 
{
    const int threadGroupSize = 1024;
    public BoidSettings boidSettings;
    public GameObject boidPrefab;
    public ComputeShader compute;
    Boid[] boids;

    void Start()
    {
        boids = new Boid[boidSettings.numBoids];
        for (int i = 0; i < boidSettings.numBoids; i++) {
            GameObject b = Instantiate(boidPrefab, transform);
            b.transform.position = new Vector3(
                boidSettings.boidRadius * (i % boidSettings.startCols), 
                boidSettings.boidRadius * Mathf.Floor(i / boidSettings.startCols),
                boidSettings.boidRadius * Mathf.Floor(i / (boidSettings.startCols * boidSettings.startRows))
            );
            boids[i] = b.GetComponent<Boid>();
            Vector3 direction = new Vector3(
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed), 
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed),
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed)
            );
            float speed = Random.Range(boidSettings.minSpeed,boidSettings.maxSpeed);
            boids[i].Init(
                boidSettings,  
                direction,
                speed
            );
        }
    }

    void Update()
    {
        if (boids != null) {
            var boidData = new BoidData[boidSettings.numBoids];

            for (int i = 0; i < boidSettings.numBoids; i++) {
                if (boids[i] != null) {
                    boidData[i].position = boids[i].position;
                    boidData[i].direction = boids[i].direction;
                }
            }

            var boidBuffer = new ComputeBuffer(boidSettings.numBoids, BoidData.Size);
            boidBuffer.SetData(boidData);

            compute.SetBuffer(0, "boids", boidBuffer);
            compute.SetInt("numBoids", boids.Length);
            compute.SetFloat("neighborMaxDist", boidSettings.neighborMaxDist);
            compute.SetFloat("desiredDist", boidSettings.desiredDist);

            int threadGroups = Mathf.CeilToInt(boidSettings.numBoids / (float) threadGroupSize);
            compute.Dispatch(0, threadGroups, 1, 1);

            boidBuffer.GetData(boidData);

            for (int i = 0; i < boids.Length; i++)
            {
                if (boids[i] != null) {
                    boids[i].flockCenter = boidData[i].flockCenter;
                    boids[i].numFlockmates = boidData[i].numFlockmates;
                    boids[i].alignmentForce = boidData[i].flockDirection;
                    boids[i].separationForce = boidData[i].separationDirection.normalized;
                    boids[i].UpdateBoid();
                }
            }

            boidBuffer.Release();
        }
    }

    public struct BoidData {
        public Vector3 position;
        public Vector3 direction;

        public Vector3 flockDirection;
        public Vector3 flockCenter;
        public Vector3 separationDirection;
        public int numFlockmates;

        public static int Size {
            get {
                return sizeof (float) * 3 * 5 + sizeof (int);
            }
        }
    }

}