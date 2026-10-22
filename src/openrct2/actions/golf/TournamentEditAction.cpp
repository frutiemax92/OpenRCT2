/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TournamentEditAction.h"

#include "../../GameState.h"
#include "../../golf/GolfRuntime.h"
#include "../../golf/Tournament.h"
#include "../../localisation/StringIds.h"

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr int32_t kMaxDaysAhead = 360;
        constexpr int64_t kMaxTicketPrice = 100000;
    } // namespace

    TournamentEditAction::TournamentEditAction(TournamentEditType type, uint8_t tier, int32_t daysAhead, int64_t ticketPrice)
        : _type(type)
        , _tier(tier)
        , _daysAhead(daysAhead)
        , _ticketPrice(ticketPrice)
    {
    }

    void TournamentEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("type", _type);
        visitor.Visit("tier", _tier);
        visitor.Visit("daysAhead", _daysAhead);
        visitor.Visit("ticketPrice", _ticketPrice);
    }

    uint16_t TournamentEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void TournamentEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_type) << DS_TAG(_tier) << DS_TAG(_daysAhead) << DS_TAG(_ticketPrice);
    }

    Result TournamentEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        auto& runtime = Golf::runtimeOf(gameState);
        auto& director = runtime.director();
        auto bad = [](StringId why = kStringIdNone) { return Result(Status::invalidParameters, STR_TOURNAMENT_CANT_CHANGE, why); };

        if (_type == TournamentEditType::cancel)
        {
            if (director.tournament.phase != Golf::TournamentPhase::scheduled)
                return bad(STR_TOURNAMENT_NONE_BOOKED);
            return Result();
        }
        if (_tier >= static_cast<uint8_t>(Golf::TournamentTier::count))
            return bad();
        if (_daysAhead < 1 || _daysAhead > kMaxDaysAhead)
            return bad(STR_TOURNAMENT_DAY_IN_THE_PAST);
        if (_ticketPrice < 0 || _ticketPrice > kMaxTicketPrice)
            return bad();
        if (director.tournament.phase == Golf::TournamentPhase::scheduled || director.tournament.phase == Golf::TournamentPhase::running)
            return bad(STR_TOURNAMENT_ALREADY_BOOKED);
        // The checklist of the tier has to be met (ECO-040); a copy of the calendar shows whether it is.
        Golf::Tournament trial = director.tournament;
        const auto result = Golf::book(
            trial, static_cast<Golf::TournamentTier>(_tier), runtime.tournamentInputs(), director.currentDay(),
            director.currentDay() + static_cast<uint32_t>(_daysAhead), _ticketPrice);
        switch (result)
        {
            case Golf::BookResult::ok:
                return Result();
            case Golf::BookResult::requirementsNotMet:
                return bad(STR_TOURNAMENT_REQUIREMENTS_NOT_MET);
            case Golf::BookResult::dayInThePast:
                return bad(STR_TOURNAMENT_DAY_IN_THE_PAST);
            case Golf::BookResult::alreadyBooked:
                return bad(STR_TOURNAMENT_ALREADY_BOOKED);
            case Golf::BookResult::negativePrice:
                return bad();
        }
        return bad();
    }

    Result TournamentEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;
        auto& runtime = Golf::runtimeOf(gameState);
        auto& director = runtime.director();
        if (_type == TournamentEditType::cancel)
        {
            Golf::cancel(director.tournament);
            return res;
        }
        Golf::book(
            director.tournament, static_cast<Golf::TournamentTier>(_tier), runtime.tournamentInputs(), director.currentDay(),
            director.currentDay() + static_cast<uint32_t>(_daysAhead), _ticketPrice);
        return res;
    }
} // namespace OpenRCT2::GameActions
