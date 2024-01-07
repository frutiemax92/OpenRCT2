#include "RideBoundboxBuilder.h"
#include "Track.h"
#include "../world/Map.h"

RideBoundboxBuilder::RideBoundboxBuilder(PreviewTrack* block, uint16_t numTiles)
{
    for (int i = 0; i < numTiles-1; i++)
    {
        auto seqBlock = block[i];
        auto seqX = Translate3DTo2DWithZ(0, { seqBlock.x, seqBlock.y, 0 }).x;
        bool found = false;
        for (const auto& elem : _offsets)
        {
            auto elemX = Translate3DTo2DWithZ(0, { elem.second.x, elem.second.y, 0 }).x;
            if (seqX == elemX)
            {
                found = true;
                break;
            }
        }

        if (!found)
            _offsets.push_back({ i, CoordsXY{ seqBlock.x, seqBlock.y } });
    }
}

std::vector<std::pair<uint32_t, CoordsXY>> RideBoundboxBuilder::GetOffsets() const
{
    return _offsets;
}
