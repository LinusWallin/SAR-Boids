
using UnityEngine;
using System.Runtime.InteropServices;

public static class OSQPSolver
{
    [DllImport("osqp_wrapper.dll")]
    private static extern void SolveCBF(
        [In] float[] position,
        [In, Out] float[] velocity,
        [In] float[] neighbors,
        [In] int numNeighbors,
        [In] float DS,
        [In] float C
    );

    public Vector3 RunOSQPSolver(Boid boid, Vector3[] neighbors, float DS, float C) {
        float[] boidPos = {
            boid.position.x, 
            boid.position.y, 
            boid.position.z
        };
        float[] boidVelocity = {
            boid.direction.x,
            boid.direction.y,
            boid.direction.z
        };

        int numNeighbors = neighbors.Length
        float[] nPos = new float[numNeighbors*3];
        for (int i = 0; i < numNeighbors; i++) {
            nPos[i * 3] = neighbors[i].x;
            nPos[i * 3 + 1] = neighbors[i].y;
            nPos[i * 3 + 2] = neighbors[i].z;
        }

        SolveCBF(boidPos, boidVelocity, nPos, numNeighbors, DS, C);

        Vector3 newVelocity = new Vector3(
            boidVelocity[0],
            boidVelocity[1],
            boidVelocity[2]
        );

        return newVelocity;
    }

}
