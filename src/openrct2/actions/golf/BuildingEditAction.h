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

#include <cstdint>

namespace OpenRCT2::GameActions
{
    enum class BuildingEditType : uint8_t
    {
        setClosed, // value = 1 closes the building to golfers, 0 opens it
        setPrice,  // value = what a visit costs, or -1 for the usual price of its kind (BLD-010)
    };

    // What the building window changes (section 9.3). The building is given by any position on it.
    class BuildingEditAction final : public GameActionBase<GameCommand::editBuilding>
    {
    private:
        CoordsXY _loc{};
        BuildingEditType _type{};
        int64_t _value{};

    public:
        BuildingEditAction() = default;
        BuildingEditAction(const CoordsXY& location, BuildingEditType type, int64_t value);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
