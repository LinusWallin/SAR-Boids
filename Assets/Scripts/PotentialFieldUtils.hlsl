
bool IsLocalMin(
        uint agentId, 
        uint bufferSize,  
        uint base,
        float revisitedDist, 
        float3 currentPos, 
        RWStructuredBuffer<float3> histBuffer,
        out float3 lmPos
    ) {
    lmPos = currentPos;
    if (bufferSize == 0)
    {
        return false;
    }
    
    for (uint t = 0; t < bufferSize; t++) {
        if (distance(currentPos, histBuffer[base + t]) < revisitedDist) {
            lmPos = histBuffer[base + t];
            return true;
        }
    }
    return false;
}
