uint3 GetGridPos(uint cellIndex, uint xMax, uint yMax) {
    uint3 g;
    g.x = cellIndex % xMax;
    g.y = (cellIndex / xMax) % yMax;
    g.z = cellIndex / (xMax * yMax);
    return g;
}

uint GetGridIndex(uint3 gridPos, uint xMax, uint yMax) {
    return gridPos.x + gridPos.y * xMax + gridPos.z * xMax * yMax;
}

uint CoordToIndex(float3 coord, float3 gridStart, float3 cellSize, uint xMax, uint yMax) {
    uint3 local = (uint3)floor((coord - gridStart) / cellSize);
    uint coordIndex = local.x + local.y * xMax + local.z * xMax * yMax;
    return coordIndex;
}

float3 IndexToCoordinate(uint cellIndex, float3 gridStart, float3 cellSize, uint xMax, uint yMax) {
    uint3 g = GetGridPos(cellIndex, xMax, yMax);
    return gridStart + float3(g.x, g.y, g.z) * cellSize;
}