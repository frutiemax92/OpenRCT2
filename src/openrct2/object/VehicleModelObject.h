/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Fixed.hpp"
#include "../model/TerrainAlignment.h"
#include "Object.h"
#include "ObjectCollision.h"

#include <cstdint>
#include <string>
#include <vector>

namespace OpenRCT2
{
    enum class VehicleModelKind : uint8_t
    {
        golfCart,
        car,
    };

    // Vehicle model object (CRT-001, STE-023).
    // Describes golf carts (CRT-001) and cars (STE-023) with seats, speeds, acceleration, contact points,
    // sounds, and ball collision geometry.
    class VehicleModelObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::vehicleModel;

        StringId NameStringId{};
        uint32_t IconImageId{};

        VehicleModelKind Kind = VehicleModelKind::golfCart;
        int32_t Seats = 2;
        Fixed MaxSpeed = Fixed::ratio(220, 100);
        Fixed Acceleration = Fixed::ratio(8, 100);
        Model::ContactPoints ContactPoints{ 10.0f, 6.0f };
        std::string EngineSound;
        std::string HornSound;
        std::string ModelPath;
        std::vector<uint8_t> ModelData; // the .glb read from the archive
        uint32_t ModelId = 0;           // in the ModelSpriteStore while the object is loaded
        ObjectCollision Collision{};

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        // Ids in the ModelSpriteStore of the loaded model of each kind, 0 while none is loaded.
        static inline uint32_t ActiveCarModelId = 0;
        static inline uint32_t ActiveCartModelId = 0;

        static VehicleModelObject* GetById(ObjectEntryIndex entryIndex);
        static VehicleModelObject* FindFirstByKind(VehicleModelKind kind);
    };
} // namespace OpenRCT2
