using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simulates the boids in the search and rescue scenario
/// </summary>
/// <author>Linus Wallin<author/>
public class Simulation : MonoBehaviour 
{
    const int threadGroupSize = 1024;
    public BoidSettings boidSettings;
    public GameObject boidPrefab;
    public Transform boundingBox;
    public ComputeShader compute;
    Boid[] boids;

    void Start()
    {
        SpawnBoids();

        foreach (Transform wallObj in boundingBox) {
            Vector3 wallCenter = wallObj.position;
            Vector3 wallSize = wallObj.gameObject.GetComponent<MeshCollider>().bounds.size;
            Vector3 wallRot = wallObj.rotation.eulerAngles;
            
            CreateCardinalMarks(wallCenter, wallSize, wallRot);
        }
    }

    void SpawnBoids ()
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
                speed,
                true
            );
        }
    }

    void CreateCardinalMarks (Vector3 center, Vector3 size, Vector3 rotation)
    {
        Debug.Log(center + ", " + size + ", " + rotation);

        Vector3 ghostDir = new Vector3();
        Vector2 planeCenter = new Vector2();
        Vector2 planeSize = new Vector2();
        string staticAxis = "";

        //floor and roof
        if (rotation == Vector3.zero || rotation == new Vector3(0, 180, 180)) {
            planeCenter = new Vector2(center.x, center.z);
            planeSize = new Vector2(size.x, size.z);
            staticAxis = "Y";
            if (rotation == Vector3.zero) {
                ghostDir = new Vector3(0, 1, 0);
            } else {
                ghostDir = new Vector3(0, -1, 0);
            }
        }
        //walls along Z-axis
        else if (rotation.x == 0 && rotation.y == 0) {
            planeCenter = new Vector2(center.z, center.y);
            planeSize = new Vector2(size.z, size.y);
            staticAxis = "X";
            if (rotation.z == 270) {
                ghostDir = new Vector3(1, 0, 0);
            } else {
                ghostDir = new Vector3(-1, 0, 0);
            }
        } 
        //walls along X-axis
        else if (rotation.y == 0 && rotation.z == 0) {
            planeCenter = new Vector2(center.x, center.y);
            planeSize = new Vector2(size.x, size.y);
            staticAxis = "Z";
            if (rotation.x == 90) {
                ghostDir = new Vector3(0, 0, 1);
            } else {
                ghostDir = new Vector3(0, 0, -1);
            }
        }

        int sepRatio = 2;

        int boidCols = (int) Mathf.Floor(planeSize.x / (sepRatio * boidSettings.boidRadius));
        int boidRows = (int) Mathf.Floor(planeSize.y / (sepRatio * boidSettings.boidRadius));
        int numGhostBoids = boidRows * boidCols;

        Vector2 botLeft = new Vector2(
            planeCenter.x - planeSize.x/2,
            planeCenter.y - planeSize.y/2
        );

        Vector2 cellSize = new Vector2(
            planeSize.x / boidCols,
            planeSize.y / boidRows
        );

        Boid[] boidCMs = new Boid[numGhostBoids];
        Vector3 boidCMPos = new Vector3();

        for (int row = 0; row < boidRows; row++) {
            for (int col = 0; col < boidCols; col++) {
                if (staticAxis == "X") {
                    boidCMPos = new Vector3(
                        center.x,
                        botLeft.y + row * cellSize.y + cellSize.y/2,
                        botLeft.x + col * cellSize.x + cellSize.x/2
                    );
                } else if (staticAxis == "Y") {
                    boidCMPos = new Vector3(
                        botLeft.x + col * cellSize.x + cellSize.x/2,
                        center.y,
                        botLeft.y + row * cellSize.y + cellSize.y/2
                    );
                } else {
                    boidCMPos = new Vector3(
                        botLeft.x + col * cellSize.x + cellSize.x/2,
                        botLeft.y + row * cellSize.y + cellSize.y/2,
                        center.z
                    );
                }
                int ghostIdx = col + col * row;
                GameObject ghostBoid = Instantiate(boidPrefab, transform);
                ghostBoid.transform.position = boidCMPos;
                boidCMs[ghostIdx] = ghostBoid.GetComponent<Boid>();
                boidCMs[ghostIdx].Init(
                    boidSettings,
                    ghostDir,
                    0,
                    false
                );
            }
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
                if (boids[i] != null & boids[i].isAlive) {
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