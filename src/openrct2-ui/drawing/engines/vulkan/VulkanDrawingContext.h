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
    // Depth-peeling transparency (matches the OpenGL renderer's SwapFramebuffer/TransparencyDepth/
    // HandleTransparency architecture exactly, see VulkanTransparencyDepth.h and
    // applytransparency_vk.vert/.frag): FilterRect/blended-or-water DrawSprite calls are routed
    // into a separate _transparentRects batch. After the opaque pass, HandleTransparency() runs
    // MaxTransparencyDepth() iterations of: draw the (unchanged) transparent batch into a
    // depth-peeling render target (using the previous iteration's depth as a "peeling" reference
    // to expose progressively deeper overlapping layers), composite it against the current opaque
    // buffer into a "mix" buffer via a fullscreen pass (applytransparency_vk), then swap which
    // physical image is "opaque" vs "mix" (ping-pong, like OpenGLFramebuffer::SwapColourBuffer)
    // ready for the next iteration or, once the loop ends, for this frame's on-screen output.
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

        // Generic image+memory+view triple, used for all the offscreen render targets below.
        struct ImageTarget
        {
            VkImage image = VK_NULL_HANDLE;
            VkDeviceMemory memory = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
        };

        // Offscreen render target that rect_vk/line_vk draw into: an 8-bit palette-index colour
        // attachment (sampled afterwards by the existing applypalette_vk composite pass) plus a
        // depth attachment used purely to establish paint order via _drawCount (see above).
        //
        // Ping-ponged between "opaque" and "mix" roles by the depth-peeling transparency loop
        // (HandleTransparency) - _currentColourIndex tracks which of the two is currently
        // "opaque" and persists across frames (safe for the dirty-rectangle LOAD_OP_LOAD scheme:
        // whichever image ends up "opaque" at the end of a frame that used transparency always
        // holds a complete, valid, full-screen image for that frame - see
        // /memories/session/transparency-implementation-progress.md for the reasoning).
        VkRenderPass _offscreenRenderPass = VK_NULL_HANDLE;
        std::array<ImageTarget, 2> _colourTargets{};
        uint32_t _currentColourIndex = 0;
        VkFormat _depthFormat = VK_FORMAT_D32_SFLOAT;
        ImageTarget _opaqueDepth;
        std::array<VkFramebuffer, 2> _opaqueFramebuffers{};

        // Mix render target: colour-only (no depth), same R8_UINT format as the opaque colour
        // targets (they're the same two physical images, just re-attached to a different render
        // pass/framebuffer depending on current role).
        VkRenderPass _mixRenderPass = VK_NULL_HANDLE;
        std::array<VkFramebuffer, 2> _mixFramebuffers{};

        // Transparent (depth-peeling) render target. Colour is 16-bit (matches OpenGL's
        // GL_R16UI transparent framebuffer - the extra bits store a "blend colour" in the upper
        // byte, see applytransparency_vk.frag) and is reused/cleared every peeling iteration.
        // Depth is ping-ponged between two physical images: iteration i draws into
        // _transparentDepthTargets[i % 2] (freshly cleared) while sampling
        // _transparentDepthTargets[(i + 1) % 2] (last iteration's depth) as the peeling
        // reference.
        VkRenderPass _transparentRenderPass = VK_NULL_HANDLE;
        ImageTarget _transparentColour;
        std::array<ImageTarget, 2> _transparentDepthTargets{};
        std::array<VkFramebuffer, 2> _transparentFramebuffers{};

        VkDescriptorSetLayout _rectDescriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout _rectPipelineLayout = VK_NULL_HANDLE;
        VkPipeline _rectPipeline = VK_NULL_HANDLE;
        VkPipeline _rectPipelineTransparent = VK_NULL_HANDLE;
        VkPipelineLayout _linePipelineLayout = VK_NULL_HANDLE;
        VkPipeline _linePipeline = VK_NULL_HANDLE;

        // Composite/apply-transparency fullscreen pass (applytransparency_vk.vert/.frag) - takes
        // opaque colour+depth, transparent colour+depth, palette and blend-palette textures and
        // writes the composited result into the mix target.
        VkDescriptorSetLayout _transparencyDescriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout _transparencyPipelineLayout = VK_NULL_HANDLE;
        VkPipeline _transparencyPipeline = VK_NULL_HANDLE;
        std::array<VkDescriptorSet, kVulkanFramesInFlight> _transparencyDescriptorSets{};

        VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
        std::array<VkDescriptorSet, kVulkanFramesInFlight> _rectDescriptorSets{};

        VkSampler _atlasSampler = VK_NULL_HANDLE;
        VkSampler _paletteSampler = VK_NULL_HANDLE;

        // Tracks what's currently actually written into each cached binding of
        // _rectDescriptorSets[frameIndex]/_transparencyDescriptorSets[frameIndex], so
        // FlushRectangles/HandleTransparency can skip a vkUpdateDescriptorSets call for any
        // binding whose bound resource hasn't changed since it was last written - the atlas,
        // palette and blend-palette textures are stable for the vast majority of frames (they
        // only change on texture-atlas growth, a rare event), and even the offscreen colour/
        // depth targets often end up referencing the same view as last time. Without this,
        // every single rendered frame (and every depth-peeling iteration within it) would pay
        // for 3-9 redundant descriptor writes for no visual benefit. VkImageView caches use
        // VK_NULL_HANDLE as an "unwritten" sentinel (real views are never null); the atlas cache
        // instead compares VulkanTextureCache::GetAtlasVersion(), since the atlas view's
        // VkImageView handle is recreated by a different class and, in principle, could be
        // reused by the driver after being destroyed. Reset to their sentinel values in Resize()
        // for the bindings that reference resize-lifetime resources.
        std::array<uint64_t, kVulkanFramesInFlight> _rectAtlasBoundVersion{};
        std::array<VkImageView, kVulkanFramesInFlight> _rectPaletteBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _rectPeelingBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositeOpaqueColourBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositeOpaqueDepthBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositeTransparentColourBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositeTransparentDepthBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositePaletteBoundView{};
        std::array<VkImageView, kVulkanFramesInFlight> _compositeBlendPaletteBoundView{};

        std::array<VulkanBuffer, kVulkanFramesInFlight> _rectInstanceBuffers{};
        std::array<VulkanBuffer, kVulkanFramesInFlight> _transparentRectInstanceBuffers{};
        std::array<VulkanBuffer, kVulkanFramesInFlight> _lineInstanceBuffers{};
        std::array<VulkanBuffer, kVulkanFramesInFlight> _stagingBuffers{};

        int32_t _drawCount = 0;
        bool _inDraw = false;
        uint32_t _ttfGlId = 0;

        RectCommandBatch _rects;
        RectCommandBatch _transparentRects;
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
            return _colourTargets[_currentColourIndex].view;
        }

        [[nodiscard]] VkImage GetColourImage() const
        {
            return _colourTargets[_currentColourIndex].image;
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
        void HandleTransparency(VkCommandBuffer cmd, uint32_t frameIndex);

        // Writes _rectDescriptorSets[frameIndex]'s bindings (atlas, palette, peeling), skipping
        // any of the three whose bound view/version already matches what's currently written -
        // see the *BoundView/_rectAtlasBoundVersion member comment.
        void UpdateRectDescriptorSetIfChanged(uint32_t frameIndex, VkImageView peelingView);

        // Writes _transparencyDescriptorSets[frameIndex]'s 6 bindings, skipping any that already
        // match what's currently written.
        void UpdateCompositeDescriptorSetIfChanged(
            uint32_t frameIndex, VkImageView opaqueColourView, VkImageView opaqueDepthView,
            VkImageView transparentColourView, VkImageView transparentDepthView);
    };
} // namespace OpenRCT2::Ui
