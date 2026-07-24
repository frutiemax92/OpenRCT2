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
#include "VulkanTextureCache.h"

#include <array>
#include <cstdint>
#include <memory>
#include <openrct2/drawing/IDrawingContext.h>
#include <vulkan/vulkan.h>

namespace OpenRCT2::Drawing
{
    struct RenderTarget;
}

namespace OpenRCT2::Ui
{
    constexpr uint32_t kVulkanFramesInFlight = 2;

    // Vulkan equivalent of the OpenGL renderer's OpenGLDrawingContext: batches draw calls into
    // CPU-side VulkanDrawRectCommand/VulkanDrawLineCommand instance buffers (see VulkanDrawCommands.h) and
    // flushes them once per frame via instanced draw calls into an offscreen R8_UINT (palette
    // index) + depth colour target. Painter's-algorithm ordering between the two instanced draw
    // calls (rects, lines) is achieved via a shared, globally-increasing depth counter converted
    // to NDC z in the vertex shaders, exactly like the OpenGL renderer - see rect_vk.vert and
    // line_vk.vert.
    //
    // Phase 1 limitation (intentional, matches the agreed plan): unlike the OpenGL renderer,
    // there is no separate depth-peeling transparency pass yet - draws that OpenGL would route
    // through its "transparent" command batch (FilterRect, blended/water sprites, hinted TTF
    // text) are instead folded into the same opaque "rects" batch here. This is a visual-only
    // regression for the "see-through rides/scenery" preference; it does not affect anything
    // else.
    class VulkanDrawingContext final : public Drawing::IDrawingContext
    {
    private:
        VkDevice _device = VK_NULL_HANDLE;
        VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
        VkQueue _queue = VK_NULL_HANDLE;
        VkCommandPool _commandPool = VK_NULL_HANDLE;

        Drawing::RenderTarget* _mainRT = nullptr;

        uint32_t _width = 0;
        uint32_t _height = 0;

        VulkanTextureCache _textureCache;

        // Offscreen render target that rect_vk/line_vk draw into: an 8-bit palette-index colour
        // attachment (sampled afterwards by the existing applypalette_vk composite pass) plus a
        // depth attachment used purely to establish paint order via _drawCount (see above).
        VkRenderPass _offscreenRenderPass = VK_NULL_HANDLE;
        VkImage _colourImage = VK_NULL_HANDLE;
        VkDeviceMemory _colourImageMemory = VK_NULL_HANDLE;
        VkImageView _colourImageView = VK_NULL_HANDLE;
        VkFormat _depthFormat = VK_FORMAT_D32_SFLOAT;
        VkImage _depthImage = VK_NULL_HANDLE;
        VkDeviceMemory _depthImageMemory = VK_NULL_HANDLE;
        VkImageView _depthImageView = VK_NULL_HANDLE;
        VkFramebuffer _offscreenFramebuffer = VK_NULL_HANDLE;

        VkDescriptorSetLayout _rectDescriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout _rectPipelineLayout = VK_NULL_HANDLE;
        VkPipeline _rectPipeline = VK_NULL_HANDLE;
        VkPipelineLayout _linePipelineLayout = VK_NULL_HANDLE;
        VkPipeline _linePipeline = VK_NULL_HANDLE;

        VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
        std::array<VkDescriptorSet, kVulkanFramesInFlight> _rectDescriptorSets{};

        VkSampler _atlasSampler = VK_NULL_HANDLE;
        VkSampler _paletteSampler = VK_NULL_HANDLE;

        std::array<VulkanBuffer, kVulkanFramesInFlight> _rectInstanceBuffers{};
        std::array<VulkanBuffer, kVulkanFramesInFlight> _lineInstanceBuffers{};
        std::array<VulkanBuffer, kVulkanFramesInFlight> _stagingBuffers{};

        int32_t _drawCount = 0;
        bool _inDraw = false;
        uint32_t _ttfGlId = 0;

        RectCommandBatch _rects;
        LineCommandBatch _lines;

        static uint8_t ComputeOutCode(ScreenCoordsXY, ScreenCoordsXY, ScreenCoordsXY);
        static bool CohenSutherlandLineClip(ScreenLine&, const Drawing::RenderTarget&);
        [[nodiscard]] ScreenRect CalculateClipping(const Drawing::RenderTarget& rt) const;

    public:
        void Initialise(
            VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool,
            Drawing::RenderTarget* mainRT);
        void Destroy();

        void Resize(uint32_t width, uint32_t height);

        void StartNewDraw();
        void FinishDraw();
        [[nodiscard]] bool IsActive() const
        {
            return _inDraw;
        }

        // Records this frame's queued texture uploads + rect/line instance buffer uploads +
        // the offscreen render pass into cmd. Must be called once per frame, after StartNewDraw
        // and after all draw calls for the frame have been issued, before the caller's own final
        // composite pass. frameIndex selects which frame-in-flight's instance/staging buffers
        // to use (matching the same double-buffering scheme as the rest of the Vulkan engine).
        void FlushCommandBuffers(VkCommandBuffer cmd, uint32_t frameIndex);

        [[nodiscard]] VkImageView GetColourImageView() const
        {
            return _colourImageView;
        }

        [[nodiscard]] VkImage GetColourImage() const
        {
            return _colourImage;
        }

        [[nodiscard]] VulkanTextureCache& GetTextureCache()
        {
            return _textureCache;
        }

        void Clear(Drawing::RenderTarget& rt, Drawing::PaletteIndex paletteIndex) override;
        void FillRect(
            Drawing::RenderTarget& rt, Drawing::PaletteIndex paletteIndex, int32_t left, int32_t top, int32_t right,
            int32_t bottom, bool crossHatch = false) override;
        void FilterRect(
            Drawing::RenderTarget& rt, Drawing::FilterPaletteID palette, int32_t left, int32_t top, int32_t right,
            int32_t bottom) override;
        void DrawLine(Drawing::RenderTarget& rt, Drawing::PaletteIndex colour, const ScreenLine& line) override;
        void DrawSprite(Drawing::RenderTarget& rt, ImageId imageId, int32_t x, int32_t y) override;
        void DrawSpriteRawMasked(
            Drawing::RenderTarget& rt, int32_t x, int32_t y, ImageId maskImage, ImageId colourImage) override;
        void DrawSpriteSolid(
            Drawing::RenderTarget& rt, ImageId image, int32_t x, int32_t y, Drawing::PaletteIndex colour) override;
        void DrawGlyph(
            Drawing::RenderTarget& rt, ImageId image, int32_t x, int32_t y, const Drawing::PaletteMap& palette) override;
        void DrawTTFBitmap(
            Drawing::RenderTarget& rt, const Drawing::TextDrawInfo& info, TTFSurface* surface, int32_t x, int32_t y,
            uint8_t hintingThreshold) override;

    private:
        void CreateOffscreenTargets();
        void DestroyOffscreenTargets();
        void CreatePipelines();
        void EnsureInstanceBufferCapacity(VulkanBuffer& buffer, VkDeviceSize requiredSize);
        void FlushRectangles(VkCommandBuffer cmd, uint32_t frameIndex);
        void FlushLines(VkCommandBuffer cmd, uint32_t frameIndex);
    };
} // namespace OpenRCT2::Ui
