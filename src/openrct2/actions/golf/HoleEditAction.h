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
    enum class HoleEditType : uint8_t
    {
        placeTee,    // index = tee category (0 men, 1 women, 2 children); turns the tile into a tee box (HOL-002)
        removeTee,   // index = tee category
        placeCup,    // on a green (HOL-003)
        removeCup,
        placeAim,    // index = aim point 0 or 1 (HOL-005)
        removeAim,   // index = aim point
        setPar,      // index = par (0 for automatic)
        setSlot,     // index = minutes between groups, 0 for Auto (HOL-020)
    };

    // Builds a hole of a course (HOL-001). When holeId is 0 a new hole is appended to the course first; the result
    // carries the id of the hole that was edited.
    class HoleEditAction final : public GameActionBase<GameCommand::editHole>
    {
    private:
        uint32_t _courseId{};
        uint32_t _holeId{};
        HoleEditType _type{};
        int32_t _index{};
        CoordsXY _loc{};

    public:
        HoleEditAction() = default;
        HoleEditAction(uint32_t courseId, uint32_t holeId, HoleEditType type, int32_t index, const CoordsXY& location = {});

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
