/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "VulkanDrawCommands.h"
#include "VulkanHelpers.h"

#include <array>
#include <cassert>
#include <cmath>
#include <openrct2/SpriteIds.h>
#include <openrct2/drawing/ImageId.hpp>
#include <unordered_map>
#include <vector>
#include <vulkan/vulkan.h>

namespace OpenRCT2::Drawing
{
    enum class FilterPaletteID : int32_t;
    struct PaletteMap;
} // namespace OpenRCT2::Drawing

namespace OpenRCT2::Ui
{
    struct VulkanGlyphId
    {
        ImageIndex Image;
        uint64_t Palette;

        struct Hash
        {
            size_t operator()(const VulkanGlyphId& k) const
            {
                size_t hash = k.Image * 7;
                hash += (k.Palette & 0xFFFFFFFFuL) * 13;
                hash += (k.Palette >> 32uL) * 23;
                return hash;
            }
        };

        struct Equal
        {
            bool operator()(const VulkanGlyphId& lhs, const VulkanGlyphId& rhs) const
            {
                return lhs.Image == rhs.Image && lhs.Palette == rhs.Palette;
            }
        };
    };

    // Same granularity as the OpenGL renderer's texture cache (see
    // src/openrct2-ui/drawing/engines/opengl/TextureCache.h) - kept identical intentionally.
    constexpr int32_t kTextureCacheMaxAtlasSize = 2048;
    constexpr int32_t kTextureCacheSmallestSlot = 32;

    struct VulkanBasicTextureInfo
    {
        int32_t index;
        FVec4 coords;
    };

    struct VulkanAtlasTextureInfo : public VulkanBasicTextureInfo
    {
        uint32_t slot;
        IVec4 bounds;
        ImageIndex image;
    };

    // Bookkeeping-only (no GPU calls) - identical algorithm to the OpenGL renderer's Atlas class.
    class VulkanAtlas final
    {
    private:
        int32_t _index = 0;
        int32_t _imageSize = 0;
        int32_t _atlasWidth = 0;
        int32_t _atlasHeight = 0;
        std::vector<uint32_t> _freeSlots;
        int32_t _cols = 0;
        int32_t _rows = 0;

    public:
        VulkanAtlas(int32_t index, int32_t imageSize)
            : _index(index)
            , _imageSize(imageSize)
        {
        }

        void Initialise(int32_t atlasWidth, int32_t atlasHeight)
        {
            _atlasWidth = atlasWidth;
            _atlasHeight = atlasHeight;

            _cols = std::max(1, _atlasWidth / _imageSize);
            _rows = std::max(1, _atlasHeight / _imageSize);

            _freeSlots.resize(static_cast<size_t>(_cols) * _rows);
            for (size_t i = 0; i < _freeSlots.size(); i++)
            {
                _freeSlots[i] = static_cast<uint32_t>(i);
            }
        }

        VulkanAtlasTextureInfo Allocate(int32_t actualWidth, int32_t actualHeight)
        {
            assert(!_freeSlots.empty());

            uint32_t slot = _freeSlots.back();
            _freeSlots.pop_back();

            auto bounds = GetSlotCoordinates(slot, actualWidth, actualHeight);

            VulkanAtlasTextureInfo info{};
            info.index = _index;
            info.slot = slot;
            info.bounds = bounds;
            info.coords = FVec4{
                static_cast<float>(bounds.x),
                static_cast<float>(bounds.y),
                static_cast<float>(_atlasWidth),
                static_cast<float>(_atlasHeight),
            };
            return info;
        }

        void Free(const VulkanAtlasTextureInfo& info)
        {
            assert(_index == info.index);
            _freeSlots.push_back(info.slot);
        }

        [[nodiscard]] bool IsImageSuitable(int32_t actualWidth, int32_t actualHeight) const
        {
            int32_t imageOrder = CalculateImageSizeOrder(actualWidth, actualHeight);
            int32_t atlasOrder = static_cast<int32_t>(std::log2(static_cast<float>(_imageSize)));
            return imageOrder == atlasOrder;
        }

        [[nodiscard]] int32_t GetFreeSlots() const
        {
            return static_cast<int32_t>(_freeSlots.size());
        }

        static int32_t CalculateImageSizeOrder(int32_t actualWidth, int32_t actualHeight)
        {
            int32_t actualSize = std::max(actualWidth, actualHeight);
            if (actualSize < kTextureCacheSmallestSlot)
            {
                actualSize = kTextureCacheSmallestSlot;
            }
            return static_cast<int32_t>(std::ceil(std::log2(static_cast<float>(actualSize))));
        }

    private:
        [[nodiscard]] IVec4 GetSlotCoordinates(uint32_t slot, int32_t actualWidth, int32_t actualHeight) const
        {
            int32_t row = static_cast<int32_t>(slot) / _cols;
            int32_t col = static_cast<int32_t>(slot) % _cols;
            return IVec4{
                _imageSize * col,
                _imageSize * row,
                _imageSize * col + actualWidth,
                _imageSize * row + actualHeight,
            };
        }
    };

    // Vulkan equivalent of the OpenGL renderer's TextureCache: a VK_FORMAT_R8_UINT
    // VK_IMAGE_VIEW_TYPE_2D_ARRAY holding one or more square atlases (one per distinct
    // power-of-2 sprite size class), plus a 256x256 R8_UINT palette-remap lookup texture.
    //
    // Unlike the OpenGL version, actual pixel uploads for newly-seen sprites/glyphs/text
    // bitmaps are not issued immediately (Vulkan has no immediate-mode texture upload) - they
    // are queued and flushed once per frame by the owning VulkanDrawingContext via
    // FlushPendingUploads(), before the frame's render pass begins.
    //
    // The atlas image is kept permanently in VK_IMAGE_LAYOUT_GENERAL (valid for both transfer
    // writes and shader sampling) to avoid needing per-frame layout transitions for a texture
    // that can be written to at unpredictable times - a deliberate simplification versus a
    // stricter TRANSFER_DST/SHADER_READ_ONLY_OPTIMAL transition scheme.
    class VulkanTextureCache final
    {
    private:
        struct PendingUpload
        {
            int32_t atlasLayer;
            IVec4 bounds;
            std::vector<uint8_t> pixels;
        };

        VkDevice _device = VK_NULL_HANDLE;
        VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
        VkQueue _queue = VK_NULL_HANDLE;
        VkCommandPool _commandPool = VK_NULL_HANDLE;

        bool _initialised = false;
        int32_t _atlasDimensions = kTextureCacheMaxAtlasSize;
        uint32_t _atlasCapacity = 0;
        uint32_t _atlasLayers = 0;
        uint32_t _atlasLayerLimit = 256;

        VkImage _atlasImage = VK_NULL_HANDLE;
        VkDeviceMemory _atlasImageMemory = VK_NULL_HANDLE;
        VkImageView _atlasImageView = VK_NULL_HANDLE;

        VkImage _paletteImage = VK_NULL_HANDLE;
        VkDeviceMemory _paletteImageMemory = VK_NULL_HANDLE;
        VkImageView _paletteImageView = VK_NULL_HANDLE;

        // Precomputed "what colour does colour A blend to with colour B" lookup (used by the
        // depth-peeling transparency composite pass for glass/one-way-glass style translucency -
        // see applytransparency_vk.frag), generated once from the same static
        // Drawing::GetBlendColourMap() table the OpenGL renderer's TextureCache uses.
        VkImage _blendPaletteImage = VK_NULL_HANDLE;
        VkDeviceMemory _blendPaletteImageMemory = VK_NULL_HANDLE;
        VkImageView _blendPaletteImageView = VK_NULL_HANDLE;

        std::vector<VulkanAtlas> _atlases;
        std::unordered_map<VulkanGlyphId, VulkanAtlasTextureInfo, VulkanGlyphId::Hash, VulkanGlyphId::Equal> _glyphTextureMap;
        std::vector<VulkanAtlasTextureInfo> _textureCache;
        std::array<uint32_t, SPR_IMAGE_LIST_END> _indexMap{};

        std::vector<PendingUpload> _pendingUploads;

    public:
        void Initialise(VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool);
        void Destroy();

        void InvalidateImage(ImageIndex image);
        VulkanBasicTextureInfo GetOrLoadImageTexture(ImageId imageId);
        VulkanBasicTextureInfo GetOrLoadGlyphTexture(ImageId imageId, const Drawing::PaletteMap& paletteMap);
        VulkanBasicTextureInfo GetOrLoadBitmapTexture(ImageIndex image, const void* pixels, size_t width, size_t height);

        // Records any queued texture uploads (new sprites/glyphs/text seen since the last
        // call) into the given command buffer using the provided per-frame staging buffer.
        // Must be called once per frame before the render pass that samples the atlas begins.
        void FlushPendingUploads(VkCommandBuffer cmd, VulkanBuffer& stagingBuffer);

        [[nodiscard]] VkImageView GetAtlasImageView() const
        {
            return _atlasImageView;
        }
        [[nodiscard]] VkImageView GetPaletteImageView() const
        {
            return _paletteImageView;
        }
        [[nodiscard]] VkImageView GetBlendPaletteImageView() const
        {
            return _blendPaletteImageView;
        }

        static int32_t PaletteToY(Drawing::FilterPaletteID palette);

    private:
        void EnlargeAtlasesImage(uint32_t newLayers);
        // Lazily generates the palette-remap lookup texture on first use (see .cpp for why this
        // must not happen eagerly during Initialise()).
        void EnsurePaletteTexture();
        void GeneratePaletteTexture();
        void GenerateBlendPaletteTexture();
        VulkanAtlasTextureInfo AllocateImage(int32_t imageWidth, int32_t imageHeight);
        VulkanAtlasTextureInfo LoadImageTexture(ImageId imageId);
        VulkanAtlasTextureInfo LoadGlyphTexture(ImageId imageId, const Drawing::PaletteMap& paletteMap);
        VulkanAtlasTextureInfo LoadBitmapTexture(ImageIndex image, const void* pixels, size_t width, size_t height);
    };
} // namespace OpenRCT2::Ui
