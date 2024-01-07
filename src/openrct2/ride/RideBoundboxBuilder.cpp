#include "RideBoundboxBuilder.h"
#include "Track.h"

RideBoundboxBuilder::RideBoundboxBuilder(PreviewTrack* block, uint16_t numTiles)
{
    for (int i = 0; i < numTiles-1; i++)
    {
        auto seqBlock = block[i];
        _offsets.push_back({ i, CoordsXY{ seqBlock.x, seqBlock.y } });
    }
}

std::vector<std::pair<uint32_t, CoordsXY>> RideBoundboxBuilder::GetOffsets() const
{
    return _offsets;
}
