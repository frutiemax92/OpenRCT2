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
    // Lays a water stream on the surface of a tile and connects it to the streams next to it (STR-001, STR-002).
    class StreamPlaceAction final : public GameActionBase<GameCommand::placeStream>
    {
    private:
        CoordsXY _loc;
        ObjectEntryIndex _streamType{};
        Direction _flowDirection{};

    public:
        StreamPlaceAction() = default;
        StreamPlaceAction(const CoordsXY& location, ObjectEntryIndex streamType, Direction flowDirection);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
