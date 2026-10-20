/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../GameAction.hpp"

namespace OpenRCT2::GameActions
{
    // Marks tiles in or out of play (HOL-007). The first edit starts from the default, which is all owned land; `reset`
    // goes back to the default.
    class CourseBoundaryAction final : public GameActionBase<GameCommand::editCourseBoundary>
    {
    private:
        CoordsXY _min{};
        CoordsXY _max{};
        uint8_t _mode{}; // 0 out of play, 1 in play, 2 reset to the default

    public:
        static constexpr uint8_t kOutOfPlay = 0;
        static constexpr uint8_t kInPlay = 1;
        static constexpr uint8_t kReset = 2;

        CourseBoundaryAction() = default;
        CourseBoundaryAction(const CoordsXY& min, const CoordsXY& max, uint8_t mode);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
