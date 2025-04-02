using UnityEngine;

public static class Utils
{
    public static Vector3 Vec3Div(Vector3 u, Vector3 v) {
        return new Vector3(
            v.x == 0 ? 0 : u.x / v.x,
            v.y == 0 ? 0 : u.y / v.y,
            v.z == 0 ? 0 : u.z / v.z
        );
    }
}