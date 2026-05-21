/// <summary>
/// Returns the closest point on a line segment defined by points a and b to a point p
/// </summary>
float3 ClosestPointOnSegment(float3 a, float3 b, float3 p)
{
    float3 ab = b - a;
    float t = dot(p - a, ab) / dot(ab, ab);
    t = saturate(t);
    return a + ab * t;
}