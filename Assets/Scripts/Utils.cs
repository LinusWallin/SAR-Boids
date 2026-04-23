using UnityEngine;

/// <summary>
/// Utility functions
/// </summary>
/// <author>Linus Wallin</author>
public static class Utils
{
    /// <summary>
    /// Performs element wise division on Vector3
    /// </summary>
    /// <param name="u">The vector to be divided</param>
    /// <param name="v">The vector to divide with</param>
    /// <returns>Resulting Vector3</returns>
    public static Vector3 Vec3Div(Vector3 u, Vector3 v) {
        return new Vector3(
            v.x == 0 ? 0 : u.x / v.x,
            v.y == 0 ? 0 : u.y / v.y,
            v.z == 0 ? 0 : u.z / v.z
        );
    }

    /// <summary>
    /// Performs element wise multiplication on Vector3
    /// </summary>
    /// <param name="u">The first vector</param>
    /// <param name="v">The second vector</param>
    /// <returns>Resulting Vector3</returns>
    public static Vector3 Vec3Mult(Vector3 u, Vector3 v) {
        return new Vector3(
            u.x * v.x,
            u.y * v.y,
            u.z * v.z
        );
    }

    /// <summary>
    /// Converts a linear index to a grid position in 3D space based on the provided grid dimensions, cell size, and grid start position.
    /// </summary>
    /// <param name="index">The linear index</param>
    /// <param name="xMax">The maximum X dimension of the grid</param>
    /// <param name="yMax">The maximum Y dimension of the grid</param>
    /// <param name="cellSize">The size of each cell in the grid</param>
    /// <param name="gridStart">The start position of the grid in world space</param>
    /// <returns>The world position corresponding to the given linear index</returns>
    public static Vector3 indexToGridPos(
        uint index, 
        int xMax, 
        int yMax, 
        Vector3 cellSize, 
        Vector3 gridStart) {

        Vector3 gridPos = new Vector3(
            (int)(index % xMax),
            (int)(index / xMax) % yMax,
            (int)(index / (xMax * yMax))
        );

        Vector3 obsPos = gridStart + new Vector3(
            gridPos.x * cellSize.x,
            gridPos.y * cellSize.y,
            gridPos.z * cellSize.z
        );

        return obsPos;
    }
}