/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Object.h"
#include "ObjectCollision.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace OpenRCT2
{
    // ANM-001: required animation clips for animals
    inline constexpr std::array<std::string_view, 4> kRequiredAnimalClips = {
        "idle",
        "walk",
        "run",
        "flee"
    };

    // Animal object (ANM-001, ANM-003, R3D-003)
    class AnimalObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::animal;

        StringId NameStringId{};
        uint32_t IconImageId{};

        std::string ModelPath;
        std::vector<uint8_t> ModelData; // the .glb read from the archive
        uint32_t ModelId = 0;           // in the ModelSpriteStore while the object is loaded
        int32_t TriangleBudget = 600; // R3D-003: animal <= 600
        std::vector<std::string> Clips;
        std::vector<std::string> AllowedSurfaces; // ANM-001: habitat
        int32_t MinGroupSize = 2;
        int32_t MaxGroupSize = 6;
        int32_t SpawnRate = 10;
        std::string SoundEffect;
        ObjectCollision Collision{};

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        bool HasClip(std::string_view name) const;
        bool ValidateRequiredClips(std::vector<std::string>* missing = nullptr) const;

        static AnimalObject* GetById(ObjectEntryIndex entryIndex);
    };
} // namespace OpenRCT2

