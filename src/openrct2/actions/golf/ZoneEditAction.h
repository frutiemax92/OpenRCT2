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
    enum class ZoneEditType : uint8_t
    {
        createLot,            // cornerA, cornerB: a cottage lot (ECO-030)
        removeLot,            // zone = the lot's id
        createTournamentZone, // cornerA, cornerB, kind = TournamentZoneKind (ECO-041)
        removeTournamentZone, // zone = the zone's id
    };

    struct ZoneEditResult
    {
        uint32_t zoneId = 0;
    };

    // Draws and removes the zones of section 22: cottage lots and the areas a tournament needs. The corners are given by
    // any position in their tile.
    class ZoneEditAction final : public GameActionBase<GameCommand::editZone>
    {
    private:
        ZoneEditType _type{};
        uint32_t _zone{};
        CoordsXY _cornerA{};
        CoordsXY _cornerB{};
        int32_t _kind{};

    public:
        ZoneEditAction() = default;
        ZoneEditAction(
            ZoneEditType type, uint32_t zoneId, const CoordsXY& cornerA = {}, const CoordsXY& cornerB = {}, int32_t kind = 0);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
