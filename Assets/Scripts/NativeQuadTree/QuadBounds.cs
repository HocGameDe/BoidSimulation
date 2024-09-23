using System;
using Unity.Mathematics;

[Serializable]
public struct QuadBounds
{
    public float2 center;
    public float2 extents;
    public float2 Size => extents * 2f;
    public float2 Haft => extents * .5f;
    public float2 Min => center - extents;
    public float2 Max => center + extents;
    public float Radius => math.length(extents);

    public QuadBounds(float2 center, float2 extents) => (this.center, this.extents) = (center, extents);
    public bool Contains(float2 point) => math.all(point >= Min) && math.all(point <= Max);
    public bool Contains(QuadBounds bounds) => math.all(bounds.Min >= Min) && math.all(bounds.Max <= Max);
    public bool Intersects(QuadBounds bounds) =>
        math.abs(center.x - bounds.center.x) <= (extents.x + bounds.extents.x) &&
        math.abs(center.y - bounds.center.y) <= (extents.y + bounds.extents.y);
    public bool ContainsCircle(float2 point) => math.lengthsq(point - center) <= math.lengthsq(Radius);
    public bool ContainsCircle(QuadBounds bounds) =>
        ContainsCircle(bounds.GetCorner(0)) && ContainsCircle(bounds.GetCorner(1)) &&
        ContainsCircle(bounds.GetCorner(2)) && ContainsCircle(bounds.GetCorner(3));
    public bool IntersectsCircle(QuadBounds bounds)
    {
        float2 closesPoint = math.clamp(center, bounds.Min, bounds.Max);
        float distanceSquared = math.lengthsq(center - closesPoint);
        return distanceSquared <= math.length(Radius);
    }
    public float2 GetCorner(int zIndexChild)
    {
        return zIndexChild switch
        {
            0 => new float2(Min.x, Max.y),
            1 => Max,
            2 => Min,
            3 => new float2(Max.x, Min.y),
            _ => throw new ArgumentOutOfRangeException(nameof(zIndexChild)),
        };
    }
    public QuadBounds GetBoundsChild(int zIndexChild)
    {
        return zIndexChild switch
        {
            0 => new QuadBounds(new float2(center.x - Haft.x, center.y + Haft.y), Haft),
            1 => new QuadBounds(center + Haft, Haft),
            2 => new QuadBounds(center - Haft, Haft),
            3 => new QuadBounds(new float2(center.x + Haft.x, center.y - Haft.y), Haft),
            _ => throw new ArgumentOutOfRangeException(nameof(zIndexChild)),
        };
    }
}