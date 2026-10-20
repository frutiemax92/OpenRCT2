/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/FlagHolder.hpp"
#include "../core/Money.hpp"
#include "Object.h"
#include "ObjectCollision.h"

#include <cstdint>
#include <string>

namespace OpenRCT2
{
    constexpr int32_t kBuildingMaxFootprint = 8;
    constexpr int32_t kBuildingViewCount = 4;

    enum class BuildingKind : uint8_t
    {
        generic,
        golfClub,
        cottage,
        foodTruck,
        drivingRangeKiosk,
        cartRental,
        restroom,
    };

    enum class BuildingService : uint8_t
    {
        food,
        drink,
        toilet,
        proShop,
    };
    using BuildingServices = FlagHolder<uint8_t, BuildingService>;

    // A building occupying a rectangular footprint of tiles (BLD-001, BLD-002). The images are the icon followed by
    // one view per camera rotation, each covering the whole footprint.
    class BuildingObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::building;

        StringId NameStringId{};
        uint32_t IconImageId{};
        uint32_t FirstViewImageId{};

        BuildingKind Kind = BuildingKind::generic;
        int32_t FootprintX = 1; // tiles, in the building's own frame (rotation 0)
        int32_t FootprintY = 1;
        int32_t Height = 32; // z units
        bool AllowSloped = false;
        // Tile of the footprint holding the entrance, and the edge it opens on (0=-x, 1=+y, 2=+x, 3=-y).
        int32_t EntranceTileX = 0;
        int32_t EntranceTileY = 0;
        uint8_t EntranceEdge = 0;
        bool HasEntrance = false;

        money64 BuildCost{};
        money64 RemoveCost{};
        money64 UpkeepPerMonth{};
        BuildingServices Services{};
        int32_t Capacity = 0;
        std::string AmbientSound; // identifier of the ambient loop (§21), empty for none
        ObjectCollision Collision{};

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        // Image of the view for a camera rotation (0 to 3).
        uint32_t getViewImageId(uint8_t rotation) const
        {
            return FirstViewImageId + (rotation & 3);
        }
        // Footprint in tiles once the building is turned by `direction` quarter turns.
        int32_t getFootprintX(uint8_t direction) const
        {
            return (direction & 1) ? FootprintY : FootprintX;
        }
        int32_t getFootprintY(uint8_t direction) const
        {
            return (direction & 1) ? FootprintX : FootprintY;
        }

        static BuildingObject* GetById(ObjectEntryIndex entryIndex);
    };
} // namespace OpenRCT2
