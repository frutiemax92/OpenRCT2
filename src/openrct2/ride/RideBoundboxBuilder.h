#pragma once

#include <vector>
#include "../world/Location.hpp"

struct PreviewTrack;

class RideBoundboxBuilder
{
public:
    RideBoundboxBuilder(PreviewTrack* block, uint16_t numTiles);
    std::vector<std::pair<uint32_t, CoordsXY>> GetOffsets() const;

private:
    std::vector<std::pair<uint32_t, CoordsXY>> _offsets;
};
