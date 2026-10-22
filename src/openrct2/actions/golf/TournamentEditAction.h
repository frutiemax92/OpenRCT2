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
    enum class TournamentEditType : uint8_t
    {
        book,   // tier, daysAhead, ticketPrice: schedule a tournament (ECO-042)
        cancel, // take the booked tournament off the calendar
    };

    // Books and cancels the tournament (section 22.5). A tournament can be booked when the checklist of its tier is met.
    class TournamentEditAction final : public GameActionBase<GameCommand::editTournament>
    {
    private:
        TournamentEditType _type{};
        uint8_t _tier{};
        int32_t _daysAhead{};
        int64_t _ticketPrice{};

    public:
        TournamentEditAction() = default;
        TournamentEditAction(TournamentEditType type, uint8_t tier = 0, int32_t daysAhead = 0, int64_t ticketPrice = 0);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
