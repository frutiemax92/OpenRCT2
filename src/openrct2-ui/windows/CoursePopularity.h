#pragma once

#include <algorithm>
#include <cstdint>
#include <openrct2/GameState.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>

namespace OpenRCT2::Ui::Windows
{
    // The course popularity (0-1000) shown by the HUD and the Course Overview window. Without a golf runtime (an old
    // theme-park save) it falls back to the park rating.
    inline uint16_t GetCoursePopularity(const GameState_t& gameState)
    {
        if (gameState.golfRuntime != nullptr)
        {
            return static_cast<uint16_t>(std::clamp<int32_t>(gameState.golfRuntime->director().popularity() * 10, 0, 1000));
        }
        return gameState.park.rating;
    }
} // namespace OpenRCT2::Ui::Windows
