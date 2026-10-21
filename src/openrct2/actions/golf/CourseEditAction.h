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
#include <string>

namespace OpenRCT2::GameActions
{
    enum class CourseEditType : uint8_t
    {
        create,    // name -> a new, closed course; the result carries its id
        remove,
        rename,    // name
        setFee,    // value = green fee in currency units
        setOpen,   // value = 0 closed, 1 open
        moveHole,  // value = hole index to move, index = destination index (HOL-004)
        removeHole,// value = hole id
        setKind,   // value = 0 regular course, 1 mini putt (MPT-001)
        setCartsAllowedOffPath, // value = 0 off, 1 on (PTH-005)
        createHole              // appends an empty hole; the result carries its id (the hole tool, HOL-001)
    };

    // What a successful course edit hands back: the course touched, and the hole created or removed if any.
    struct CourseEditResult
    {
        uint32_t courseId = 0;
        uint32_t holeId = 0;
    };

    // Creates and changes golf courses (HOL-010, HOL-004).
    class CourseEditAction final : public GameActionBase<GameCommand::editCourse>
    {
    private:
        CourseEditType _type{};
        uint32_t _courseId{};
        std::string _name;
        int64_t _value{};
        int32_t _index{};

    public:
        CourseEditAction() = default;
        CourseEditAction(CourseEditType type, uint32_t courseId, const std::string& name = {}, int64_t value = 0, int32_t index = 0);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
