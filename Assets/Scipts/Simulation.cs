using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Simulates the boids in the search and rescue scenario
/// </summary>
/// <author>Linus Wallin<author/>
public class Simulation : MonoBehaviour 
{
    const int threadGroupSize = 1024;
    public BoidSettings boidSettings;
    public GameObject boidPrefab;
    public GameObject[] obstacles;
    public Transform boundingBox;
    public ComputeShader compute;
    [SerializeField] private GameObject target;
    [SerializeField] private LayerMask obstacleMask;
    Boid[] boids;
    Boid[] aliveBoids;
    Boid[] boidCMs;

    void Start()
    {
        SpawnBoids();

        List<Boid[]> boidList = new List<Boid[]>();
        int numGhosts = 0;

        //Loops through the planes of the bounding box and places ghosts on them
        foreach (Transform wallObj in boundingBox) {
            Vector3 wallCenter = wallObj.position;
            Vector3 wallSize = wallObj.gameObject.GetComponent<MeshCollider>().bounds.size;
            Vector3 wallRot = wallObj.rotation.eulerAngles;
            
            Boid[] planeCMs = CreateCardinalMarks(wallCenter, wallSize, wallRot);
            boidList.Add(planeCMs);
            numGhosts += planeCMs.Length;
        }

        boidCMs = new Boid[numGhosts];

        //merging the arrays of the different planes into one ghost boid array
        int arrIndex = 0;
        foreach (Boid[] boidArr in boidList) {
            for (int i = 0; i < boidArr.Length; i++) {
                boidCMs[arrIndex] = boidArr[i];
                arrIndex++;
            }
        }

        //Isolated Danger Marks
        List<Boid> boidIDMs = new List<Boid>();
        foreach (GameObject obs in obstacles) {
            boidIDMs = CreateIDMs(obs, boidIDMs);
        }
        Boid[] arrayIDM = boidIDMs.ToArray();
        int numIDMs = arrayIDM.Length;

        boids = new Boid[boidSettings.numBoids + numGhosts + numIDMs];
        
        for (int i = 0; i < boidSettings.numBoids; i++) {
            boids[i] = aliveBoids[i];
        }

        for (int j = 0; j < numGhosts; j++) {
            boids[boidSettings.numBoids + j] = boidCMs[j];
        }

        for (int k = 0; k < numIDMs; k++) {
            boids[boidSettings.numBoids + numGhosts + k] = arrayIDM[k];
        }

    }

    void SpawnBoids ()
    {
        aliveBoids = new Boid[boidSettings.numBoids + boidSettings.leaders];
        for (int i = 0; i < boidSettings.numBoids; i++) {
            GameObject b = Instantiate(boidPrefab, transform);
            b.transform.position = new Vector3(
                boidSettings.boidRadius * (i % boidSettings.startCols), 
                boidSettings.boidRadius * Mathf.Floor(i / boidSettings.startCols),
                boidSettings.boidRadius * Mathf.Floor(i / (boidSettings.startCols * boidSettings.startRows))
            );
            aliveBoids[i] = b.GetComponent<Boid>();
            Vector3 direction = new Vector3(
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed), 
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed),
                Random.Range(-boidSettings.maxSpeed, boidSettings.maxSpeed)
            );
            float speed = Random.Range(boidSettings.minSpeed,boidSettings.maxSpeed);
            aliveBoids[i].Init(
                boidSettings,  
                direction,
                speed,
                true
            );
        }
        int[] leaderIndices = RandomBoidSubset(aliveBoids.Length, boidSettings.leaders);
        foreach (int leaderIdx in leaderIndices) {
            Boid leaderBoid = aliveBoids[leaderIdx].GetComponent<Boid>();
            leaderBoid.isLeader = true;
            leaderBoid.target = target;
        }
    }

    private int[] RandomBoidSubset(int arrLen, int subsetSize) {
        HashSet<int> indices = new HashSet<int>();
        int[] subset = new int[subsetSize];

        while (indices.Count < subsetSize) {
            int idx = Random.Range(0, arrLen - 1);
            if (indices.Add(idx)) {
                subset[indices.Count - 1] = idx;
            }
        }

        return subset;
    }

    /// <summary>
    /// Spawns ghost boids on a plane with a given center point, size and rotation
    /// </summary>
    /// <param name="center">Center point of the plane</param>
    /// <param name="size">Size of the plane</param>
    /// <param name="rotation">Rotation of the plane</param>
    /// <returns>returns the boid array of ghosts</returns>
    Boid[] CreateCardinalMarks (Vector3 center, Vector3 size, Vector3 rotation)
    {

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

        int boidCols = (int) Mathf.Floor(planeSize.x / (boidSettings.sepRatio * boidSettings.boidRadius));
        int boidRows = (int) Mathf.Floor(planeSize.y / (boidSettings.sepRatio * boidSettings.boidRadius));
        int numGhostBoids = boidRows * boidCols;

        Vector2 botLeft = new Vector2(
            planeCenter.x - planeSize.x/2,
            planeCenter.y - planeSize.y/2
        );

        Vector2 cellSize = new Vector2(
            planeSize.x / boidCols,
            planeSize.y / boidRows
        );

        Boid[] boidCM = new Boid[numGhostBoids];
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
                boidCM[ghostIdx] = ghostBoid.GetComponent<Boid>();
                boidCM[ghostIdx].Init(
                    boidSettings,
                    ghostDir,
                    0,
                    false
                );
            }
        }
        return boidCM;

    }


    /// <summary>
    /// Places ghost boids in a grid formation on the faces
    /// of a obstacle.
    /// </summary>
    /// <param name="obstacle">The obstacle which the ghost boids should be placed on</param>
    /// <param name="boidIDMs">IDMs List to keep track of ghost boids</param>
    /// <returns>Returns updated IDM List</returns>
    List<Boid> CreateIDMs(GameObject obstacle, List<Boid> boidIDMs) {
        MeshCollider meshCollider = obstacle.GetComponent<MeshCollider>();
        Bounds bounds =  meshCollider.sharedMesh.bounds; //Gets local bounds of obstacle
        Vector3 center = bounds.center;
        Vector3 extents = bounds.extents;
        Vector3 gridExtents = Vector3.Scale(extents, obstacle.transform.lossyScale);
        Vector3 boidsInDir = (gridExtents*2)/(boidSettings.boidRadius*boidSettings.sepRatio);
        boidsInDir = new Vector3(
            Mathf.Floor(boidsInDir.x),
            Mathf.Floor(boidsInDir.y),
            Mathf.Floor(boidsInDir.z)
        );

        // Faces of game object
        // left, right, top, bottom, front, back
        // center, normal, row direction, column direction
        FaceData[] faces = new FaceData[] {
            new FaceData(
                obstacle,
                center + new Vector3(extents.x, 0, 0),
                Vector3.right,
                Vector3.forward,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center - new Vector3(extents.x, 0, 0),
                Vector3.left,
                Vector3.back,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center + new Vector3(0, extents.y, 0),
                Vector3.up,
                Vector3.right,
                Vector3.back
            ),
            new FaceData(
                obstacle,
                center - new Vector3(0, extents.y, 0),
                Vector3.down,
                Vector3.left,
                Vector3.back
            ),
            new FaceData(
                obstacle,
                center + new Vector3(0, 0, extents.z),
                Vector3.forward,
                Vector3.right,
                Vector3.up
            ),
            new FaceData(
                obstacle,
                center - new Vector3(0, 0, extents.z),
                Vector3.back,
                Vector3.left,
                Vector3.up
            )
        };

        foreach (FaceData face in faces) {
            Vector3 faceCenter = obstacle.transform.InverseTransformPoint(face.center);
            Vector3 normal = obstacle.transform.InverseTransformDirection(face.normal);
            Vector3 rowDir = obstacle.transform.InverseTransformDirection(face.rowDir);
            Vector3 colDir = obstacle.transform.InverseTransformDirection(face.colDir);

            Vector3 rowStepSize = Utils.Vec3Div(rowDir, obstacle.transform.lossyScale) * (boidSettings.boidRadius * boidSettings.sepRatio);
            Vector3 colStepSize = Utils.Vec3Div(colDir, obstacle.transform.lossyScale) * (boidSettings.boidRadius * boidSettings.sepRatio);
            Vector3 rowExt = Vector3.Scale(rowDir, extents);
            Vector3 colExt = Vector3.Scale(colDir, extents);
            Vector3 botLeft = faceCenter - rowExt - colExt;

            int numRows = Mathf.Abs((int)Vector3.Dot(rowDir, boidsInDir));
            int numCols = Mathf.Abs((int)Vector3.Dot(colDir, boidsInDir));
            
            Vector3 gridStart = botLeft;

            if (numRows == 0) {
                gridStart += rowExt;
                numRows = 1;
            } else {
                gridStart += rowStepSize / 2;
            }
            if (numCols == 0) {
                gridStart += colExt;
                numCols = 1;
            } else {
                gridStart += colStepSize / 2;
            }
            
            for (int x = 0; x < numRows; x++) {
                for (int y = 0; y < numCols; y++) {
                    Vector3 posIDM = gridStart + rowStepSize * x + colStepSize * y;
                    GameObject ghostBoid = Instantiate(boidPrefab, transform);
                    RaycastHit hit;
                    Vector3 surfaceNormal = normal;
                    Vector3 globalPos = obstacle.transform.TransformPoint(posIDM);
                    Vector3 globalNormal = obstacle.transform.TransformDirection(normal);
                    if (Physics.Raycast(globalPos + globalNormal, -globalNormal, out hit, obstacleMask)) {
                        surfaceNormal = hit.normal;
                        posIDM = hit.point;
                        Debug.DrawLine(globalPos + globalNormal, hit.point, Color.red, 10000);
                    }

                    ghostBoid.transform.position = posIDM;
                    Vector3 boidScale = boidPrefab.transform.localScale;
                    float maxScale = Mathf.Max(
                        Mathf.Max(
                            obstacle.transform.localScale.x,
                            obstacle.transform.localScale.y
                        ),
                        obstacle.transform.localScale.z
                    );
                    boidIDMs.Add(ghostBoid.GetComponent<Boid>());
                    boidIDMs[boidIDMs.Count-1].Init(
                        boidSettings,
                        surfaceNormal,
                        0,
                        false
                    );
                }
            }
        }
        return boidIDMs;
    }

    void Update()
    {
        if (boids != null) {
            var boidData = new BoidData[boids.Length];

            for (int i = 0; i < boids.Length; i++) {
                if (boids[i] != null) {
                    boidData[i].position = boids[i].position;
                    boidData[i].direction = boids[i].direction;
                    if (boids[i].isAlive) {
                        boidData[i].isAlive = 1;
                    } else {
                        boidData[i].isAlive = 0;
                    }
                }
            }

            var boidBuffer = new ComputeBuffer(boids.Length, BoidData.Size);
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
                if (boids[i] != null && boids[i].isAlive) {
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

    public struct FaceData {
        public GameObject obstacle;
        public Vector3 center;
        public Vector3 normal;
        public Vector3 rowDir;
        public Vector3 colDir;

        public FaceData(GameObject obstacle, Vector3 center, Vector3 normal, Vector3 rowDir, Vector3 colDir) {
            this.obstacle = obstacle;
            this.center = obstacle.transform.TransformPoint(center);
            this.normal = obstacle.transform.TransformDirection(normal);
            this.rowDir = obstacle.transform.TransformDirection(rowDir);
            this.colDir = obstacle.transform.TransformDirection(colDir);
        }
    }

    public struct BoidData {
        public Vector3 position;
        public Vector3 direction;

        public Vector3 flockDirection;
        public Vector3 flockCenter;
        public Vector3 separationDirection;
        public int numFlockmates;
        public int isAlive;

        public static int Size {
            get {
                return sizeof (float) * 3 * 5 + sizeof (int) * 2;
            }
        }
    }

}