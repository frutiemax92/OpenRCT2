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
    enum class RangeEditType : uint8_t
    {
        create,           // cornerA, cornerB -> a new driving range; the result carries its id (DRV-001)
        remove,
        setArea,          // cornerA, cornerB
        addHittingTile,   // cornerA is the tile (DRV-001)
        removeHittingTile,
        setPrice,         // value = the price of a bucket (DRV-005)
        setBalls,         // value = balls in a bucket, 1..100 (DRV-005)
        setDirection,     // value = degrees 0..359 golfers shoot in; the default is towards the middle of the area
        autoDirection,    // back to aiming towards the middle of the area
    };

    struct RangeEditResult
    {
        uint32_t rangeId = 0;
    };

    // Creates and changes driving ranges (section 17). Tiles are given by any position inside them.
    class RangeEditAction final : public GameActionBase<GameCommand::editRange>
    {
    private:
        RangeEditType _type{};
        uint32_t _rangeId{};
        CoordsXY _cornerA{};
        CoordsXY _cornerB{};
        int64_t _value{};

    public:
        RangeEditAction() = default;
        RangeEditAction(
            RangeEditType type, uint32_t rangeId, const CoordsXY& cornerA = {}, const CoordsXY& cornerB = {}, int64_t value = 0);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
