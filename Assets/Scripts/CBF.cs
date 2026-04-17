using UnityEngine;
using System.Runtime.InteropServices;
using System.Collections.Generic;

public class CBF : MonoBehaviour {
    [DllImport("Wrapper")]
    private static extern void SolveCBF (
        float[] pos,
        float[] vel,
        float[] neighbors,
        int num_neighbors,
        float ds,
        float c        
    );

    public void RunCBF (Vector3 pos, Vector3 vel, List<Boid> neighbors, float CBF_DS, float CBF_C) {
        int numNeighbors = neighbors.Count;
        float[] flatNeighbors = new float[numNeighbors * 3];
        float[] posArr = {pos.x, pos.y, pos.z};
        float[] velArr = {vel.x, vel.y, vel.z};
        
        for (int i=0; i<numNeighbors; i++) {
            flatNeighbors[i * 3] = neighbors[i].direction.x;
            flatNeighbors[i * 3 + 1] = neighbors[i].direction.y;
            flatNeighbors[i * 3 + 2] = neighbors[i].direction.z;
        }
        SolveCBF(posArr, velArr, flatNeighbors, numNeighbors, CBF_DS, CBF_C);

        Vector2 velocity = new Vector2(velArr[0], velArr[1]).normalized;
    }
}