/// <summary>
/// Converts a cell index to a 3D coordinate in local space
/// </summary>
/// <param name="cellIndex">the cell index to convert</param>
/// <param name="xMax">max number of cells in x-direction</param>
/// <param name="yMax">max number of cells in y-direction</param>
uint3 GetGridPos(uint cellIndex, uint xMax, uint yMax) {
    uint3 g;
    g.x = cellIndex % xMax;
    g.y = (cellIndex / xMax) % yMax;
    g.z = cellIndex / (xMax * yMax);
    return g;
}

/// <summary>
/// Converts a position into an index in the grid space
/// </summary>
/// <param name="gridPos">the position in 3D space</param>
/// <param name="xMax">max number of cells in x-direction</param>
/// <param name="yMax">max number of cells in y-direction</param>
uint GetGridIndex(uint3 gridPos, uint xMax, uint yMax) {
    return gridPos.x + gridPos.y * xMax + gridPos.z * xMax * yMax;
}

/// <summary>
/// Converts a position into an index in the local grid space
/// </summary>
/// <param name="coord">the coordinate of the position in 3D space</param>
/// <param name="gridStart">float3 that defines the first cell position in global space</param>
/// <param name="cellSize">size of each cell in the grid</param>
/// <param name="xMax">max number of cells in x-direction</param>
/// <param name="yMax">max number of cells in y-direction</param>
uint CoordToIndex(float3 coord, float3 gridStart, float3 cellSize, uint xMax, uint yMax) {
    uint3 local = (uint3)floor((coord - gridStart) / cellSize);
    uint coordIndex = local.x + local.y * xMax + local.z * xMax * yMax;
    return coordIndex;
}

/// <summary>
/// Converts a cell index to a 3D coordinate in global space
/// </summary>
/// <param name="cellIndex">the cell index to convert</param>
/// <param name="gridStart">float3 that defines the first cell position in global space</param>
/// <param name="xMax">max number of cells in x-direction</param>
/// <param name="yMax">max number of cells in y-direction</param>
float3 IndexToCoordinate(uint cellIndex, float3 gridStart, float3 cellSize, uint xMax, uint yMax) {
    uint3 g = GetGridPos(cellIndex, xMax, yMax);
    return gridStart + float3(g.x, g.y, g.z) * cellSize;
}