/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../world/TerrainShape.h"
#include "../GameAction.hpp"

namespace OpenRCT2::GameActions
{
    class LandSetHeightAction final : public GameActionBase<GameCommand::setLandHeight>
    {
    private:
        CoordsXY _coords;
        uint8_t _height{};
        uint16_t _shape{}; // encoded Terrain::TerrainShape, offsets in Q above the height

        uint8_t GetHighestOffset() const;

    public:
        LandSetHeightAction() = default;
        // The height is in Q (8 z units); the shape holds the corner offsets in Q.
        LandSetHeightAction(const CoordsXY& coords, uint8_t height, const Terrain::TerrainShape& shape);
        // Legacy slope bit flags.
        LandSetHeightAction(const CoordsXY& coords, uint8_t height, uint8_t style);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;

    private:
        StringId CheckParameters() const;
        TileElement* CheckTreeObstructions() const;
        money64 GetSmallSceneryRemovalCost() const;
        void SmallSceneryRemoval() const;
        StringId CheckRideSupports() const;
        TileElement* CheckFloatingStructures(TileElement* surfaceElement, uint8_t zCorner) const;
        money64 GetSurfaceHeightChangeCost(SurfaceElement* surfaceElement) const;
        void SetSurfaceHeight(TileElement* surfaceElement) const;

        /**
         *
         *  rct2: 0x00663CB9
         */
        static bool MapSetLandHeightClearFunc(
            TileElement** tile_element, [[maybe_unused]] const CoordsXY& coords, [[maybe_unused]] CommandFlags flags,
            [[maybe_unused]] money64* price);
    };
} // namespace OpenRCT2::GameActions
