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
    // Lays a tile of street on the ground and connects it to the streets next to it (STE-001, STE-002). The slope of the
    // road follows the ramp of the terrain under it (STE-003); a one-way street flows towards `flowDirection`.
    class StreetPlaceAction final : public GameActionBase<GameCommand::placeStreet>
    {
    private:
        CoordsXY _loc;
        ObjectEntryIndex _streetType{};
        Direction _flowDirection{};

    public:
        StreetPlaceAction() = default;
        StreetPlaceAction(const CoordsXY& location, ObjectEntryIndex streetType, Direction flowDirection = 0);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
