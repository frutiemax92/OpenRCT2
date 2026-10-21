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
    // Makes a street tile at the edge of the map a road entrance, where cars come in and leave (STE-005), or takes the
    // entrance away again. The scenario editor places them; at least one is needed to receive car traffic.
    class StreetRoadEntranceAction final : public GameActionBase<GameCommand::setRoadEntrance>
    {
    private:
        CoordsXY _loc;
        bool _entrance{};

    public:
        StreetRoadEntranceAction() = default;
        StreetRoadEntranceAction(const CoordsXY& location, bool entrance);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
