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
    // Lays a tile of parking on level ground (STE-010): two spaces facing `facing`, the edge the cars drive in through. The
    // surface is the road texture of `streetType`. With `cartBay` the tile is a bay for one golf cart instead (CRT-002).
    class ParkingPlaceAction final : public GameActionBase<GameCommand::placeParking>
    {
    private:
        CoordsXY _loc;
        ObjectEntryIndex _streetType{};
        Direction _facing{};
        bool _cartBay{};

    public:
        ParkingPlaceAction() = default;
        ParkingPlaceAction(const CoordsXY& location, ObjectEntryIndex streetType, Direction facing, bool cartBay = false);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
