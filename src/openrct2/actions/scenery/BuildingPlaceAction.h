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
    // Puts a building on the ground: `_loc` is the lowest-x, lowest-y corner of its footprint once turned (BLD-001).
    class BuildingPlaceAction final : public GameActionBase<GameCommand::placeBuilding>
    {
    private:
        CoordsXY _loc;
        ObjectEntryIndex _buildingType{};
        Direction _rotation{};

    public:
        BuildingPlaceAction() = default;
        BuildingPlaceAction(const CoordsXY& location, ObjectEntryIndex buildingType, Direction rotation);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
