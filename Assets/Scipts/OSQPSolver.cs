
using UnityEngine;
using System.Runtime.InteropServices;

public static class OSQPSolver
{
    /// <summary>
    /// Calls the osqp_wrapper function which runs the C code and OSQP
    /// </summary>
    /// <param name="position">The position of the boid</param>
    /// <param name="velocity">The velocity of the boid</param>
    /// <param name="neighbors">The positions of neighbors of the boid</param>
    /// <param name="numNeighbors">Number of neighbors for the boid</param>
    /// <param name="DS">Minimum allowed distance</param>
    /// <param name="C"></param>
    [DllImport("osqp_wrapper.dll")]
    private static extern void SolveCBF(
        [In] float[] position,
        [In, Out] float[] velocity,
        [In] float[] neighbors,
        [In] int numNeighbors,
        [In] float DS,
        [In] float C
    );

    /// <summary>
    /// Takes input from the simulation and translates it to the correct format for the OSQP solver
    /// </summary>
    /// <param name="boid">The current boid</param>
    /// <param name="neighbors">The neighbors of the boid</param>
    /// <param name="DS">Minimum allowed distance</param>
    /// <param name="C"></param>
    /// <returns>Returns updated velocity from OSQP solution</returns>
    public static Vector3 RunOSQPSolver(Boid boid, float DS, float C) {
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

        int numNeighbors = boid.neighborPos.Count;
        Debug.Log(numNeighbors);
        float[] nPos = new float[numNeighbors*3];
        for (int i = 0; i < numNeighbors; i++) {
            Vector3 n = boid.neighborPos[i];
            nPos[i * 3] = n.x;
            nPos[i * 3 + 1] = n.y;
            nPos[i * 3 + 2] = n.z;
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
