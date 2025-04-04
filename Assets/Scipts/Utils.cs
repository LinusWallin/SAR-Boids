using UnityEngine;

/// <summary>
/// Utility functions
/// </summary>
/// <author>Linus Wallin<author/>
public static class Utils
{
    /// <summary>
    /// Performes element wise division on Vector3
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
}