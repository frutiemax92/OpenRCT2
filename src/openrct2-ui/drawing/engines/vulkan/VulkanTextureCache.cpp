/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#ifndef DISABLE_VULKAN

#include "VulkanTextureCache.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <openrct2/core/EnumUtils.hpp>
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/FilterPaletteIds.h>
#include <openrct2/drawing/RenderTarget.h>
#include <stdexcept>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

namespace
{
    constexpr uint32_t kUnusedIndex = 0xFFFFFFFF;

    void RunOneTimeCommands(
        VkDevice device, VkQueue queue, VkCommandPool commandPool, const std::function<void(VkCommandBuffer)>& fn)
    {
        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = commandPool;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = 1;

        VkCommandBuffer cmd = VK_NULL_HANDLE;
        CheckVk(vkAllocateCommandBuffers(device, &alloc, &cmd), "vkAllocateCommandBuffers(texcache one-time)");

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        CheckVk(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer(texcache one-time)");

        fn(cmd);

        CheckVk(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(texcache one-time)");

        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        CheckVk(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE), "vkQueueSubmit(texcache one-time)");
        CheckVk(vkQueueWaitIdle(queue), "vkQueueWaitIdle(texcache one-time)");

        vkFreeCommandBuffers(device, commandPool, 1, &cmd);
    }

    // Purely CPU-side helpers, identical to the OpenGL renderer's TextureCache::CreateRT/DeleteRT
    // - allocate a small scratch palette-index buffer, decode the sprite into it via the existing
    // software rasteriser (GfxDrawSpriteSoftware/GfxDrawSpritePaletteSetSoftware), then discard.
    RenderTarget CreateScratchRT(int32_t width, int32_t height)
    {
        size_t numPixels = static_cast<size_t>(width) * height;
        auto pixels8 = new PaletteIndex[numPixels];
        std::fill_n(pixels8, numPixels, PaletteIndex::transparent);

        RenderTarget rt;
        rt.bits = pixels8;
        rt.pitch = 0;
        rt.x = 0;
        rt.y = 0;
        rt.width = width;
        rt.height = height;
        rt.zoom_level = ZoomLevel{ 0 };
        return rt;
    }

    void DeleteScratchRT(RenderTarget& rt)
    {
        delete[] rt.bits;
        rt.bits = nullptr;
    }
} // namespace

void VulkanTextureCache::Initialise(VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool)
{
    _device = device;
    _physicalDevice = physicalDevice;
    _queue = queue;
    _commandPool = commandPool;

    std::fill(_indexMap.begin(), _indexMap.end(), kUnusedIndex);

    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(physicalDevice, &props);
    _atlasDimensions = std::min<int32_t>(kTextureCacheMaxAtlasSize, static_cast<int32_t>(props.limits.maxImageDimension2D));
    _atlasLayerLimit = std::min<uint32_t>(256, props.limits.maxImageArrayLayers);

    // Deliberately NOT generating the palette texture here. Doing so eagerly (as part of engine
    // Initialise(), which runs very early during a cold boot with Vulkan as the initially
    // configured renderer) can run before all G1/object data used for palette remaps (ride and
    // vehicle recolours, peep clothing, etc.) has finished loading, permanently baking an
    // incomplete lookup texture that is never regenerated afterwards. Instead this is deferred
    // until the first actual texture request (see EnsurePaletteTexture()), matching the OpenGL
    // renderer's equally lazy TextureCache::CreateTextures() behaviour.
}

void VulkanTextureCache::EnsurePaletteTexture()
{
    if (_initialised)
        return;

    GeneratePaletteTexture();
    _initialised = true;
}

void VulkanTextureCache::Destroy()
{
    if (_device == VK_NULL_HANDLE)
        return;

    if (_atlasImageView != VK_NULL_HANDLE)
        vkDestroyImageView(_device, _atlasImageView, nullptr);
    if (_atlasImage != VK_NULL_HANDLE)
        vkDestroyImage(_device, _atlasImage, nullptr);
    if (_atlasImageMemory != VK_NULL_HANDLE)
        vkFreeMemory(_device, _atlasImageMemory, nullptr);

    if (_paletteImageView != VK_NULL_HANDLE)
        vkDestroyImageView(_device, _paletteImageView, nullptr);
    if (_paletteImage != VK_NULL_HANDLE)
        vkDestroyImage(_device, _paletteImage, nullptr);
    if (_paletteImageMemory != VK_NULL_HANDLE)
        vkFreeMemory(_device, _paletteImageMemory, nullptr);

    _atlasImageView = VK_NULL_HANDLE;
    _atlasImage = VK_NULL_HANDLE;
    _atlasImageMemory = VK_NULL_HANDLE;
    _paletteImageView = VK_NULL_HANDLE;
    _paletteImage = VK_NULL_HANDLE;
    _paletteImageMemory = VK_NULL_HANDLE;
    _device = VK_NULL_HANDLE;
}

void VulkanTextureCache::InvalidateImage(ImageIndex image)
{
    uint32_t index = _indexMap[image];
    if (index == kUnusedIndex)
        return;

    VulkanAtlasTextureInfo& elem = _textureCache.at(index);
    _atlases[elem.index].Free(elem);
    _indexMap[image] = kUnusedIndex;

    if (index == _textureCache.size() - 1)
    {
        _textureCache.pop_back();
    }
    else
    {
        VulkanAtlasTextureInfo& last = _textureCache.back();
        elem = last;
        _indexMap[last.image] = index;
        _textureCache.pop_back();
    }
}

VulkanBasicTextureInfo VulkanTextureCache::GetOrLoadImageTexture(ImageId imageId)
{
    EnsurePaletteTexture();

    uint32_t index = _indexMap[imageId.GetIndex()];
    if (index != kUnusedIndex)
    {
        const auto& info = _textureCache[index];
        return { info.index, info.coords };
    }

    index = static_cast<uint32_t>(_textureCache.size());
    VulkanAtlasTextureInfo info = LoadImageTexture(imageId);
    _textureCache.push_back(info);
    _indexMap[imageId.GetIndex()] = index;
    return info;
}

VulkanBasicTextureInfo VulkanTextureCache::GetOrLoadGlyphTexture(ImageId imageId, const PaletteMap& paletteMap)
{
    EnsurePaletteTexture();

    VulkanGlyphId glyphId{};
    glyphId.Image = imageId.GetIndex();

    PaletteIndex glyphMap[8];
    for (uint8_t i = 0; i < 8; i++)
    {
        glyphMap[i] = paletteMap[i];
    }
    std::copy_n(glyphMap, sizeof(glyphId.Palette), reinterpret_cast<PaletteIndex*>(&glyphId.Palette));

    auto kvp = _glyphTextureMap.find(glyphId);
    if (kvp != _glyphTextureMap.end())
    {
        return { kvp->second.index, kvp->second.coords };
    }

    auto cacheInfo = LoadGlyphTexture(imageId, paletteMap);
    auto it = _glyphTextureMap.insert(std::make_pair(glyphId, cacheInfo));
    return { it.first->second.index, it.first->second.coords };
}

VulkanBasicTextureInfo VulkanTextureCache::GetOrLoadBitmapTexture(ImageIndex image, const void* pixels, size_t width, size_t height)
{
    EnsurePaletteTexture();

    uint32_t index = _indexMap[image];
    if (index != kUnusedIndex)
    {
        const auto& info = _textureCache[index];
        return { info.index, info.coords };
    }

    index = static_cast<uint32_t>(_textureCache.size());
    VulkanAtlasTextureInfo info = LoadBitmapTexture(image, pixels, width, height);
    _textureCache.push_back(info);
    _indexMap[image] = index;
    return info;
}

VulkanAtlasTextureInfo VulkanTextureCache::AllocateImage(int32_t imageWidth, int32_t imageHeight)
{
    for (VulkanAtlas& atlas : _atlases)
    {
        if (atlas.GetFreeSlots() > 0 && atlas.IsImageSuitable(imageWidth, imageHeight))
        {
            return atlas.Allocate(imageWidth, imageHeight);
        }
    }

    if (static_cast<uint32_t>(_atlases.size()) >= _atlasLayerLimit)
    {
        throw std::runtime_error("Vulkan texture cache: more texture atlases required, but device limit reached!");
    }

    auto atlasIndex = static_cast<int32_t>(_atlases.size());
    int32_t atlasSize = static_cast<int32_t>(
        std::pow(2.0f, static_cast<float>(VulkanAtlas::CalculateImageSizeOrder(imageWidth, imageHeight))));

    _atlases.emplace_back(atlasIndex, atlasSize);
    _atlases.back().Initialise(_atlasDimensions, _atlasDimensions);

    EnlargeAtlasesImage(1);

    return _atlases.back().Allocate(imageWidth, imageHeight);
}

void VulkanTextureCache::EnlargeAtlasesImage(uint32_t newLayers)
{
    uint32_t newLayerCount = _atlasLayers + newLayers;
    if (newLayerCount <= _atlasCapacity)
    {
        _atlasLayers = newLayerCount;
        return;
    }

    // Initial capacity of 12 covers most cases of a fully visible park (mirrors the OpenGL
    // texture cache's growth policy).
    uint32_t newCapacity = (_atlasCapacity + 6) << 1u;

    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8_UINT;
    imageInfo.extent = { static_cast<uint32_t>(_atlasDimensions), static_cast<uint32_t>(_atlasDimensions), 1 };
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = newCapacity;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VkImage newImage = VK_NULL_HANDLE;
    CheckVk(vkCreateImage(_device, &imageInfo, nullptr, &newImage), "vkCreateImage(atlas)");

    VkMemoryRequirements memReq{};
    vkGetImageMemoryRequirements(_device, newImage, &memReq);

    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = memReq.size;
    alloc.memoryTypeIndex = FindMemoryType(_physicalDevice, memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    VkDeviceMemory newMemory = VK_NULL_HANDLE;
    CheckVk(vkAllocateMemory(_device, &alloc, nullptr, &newMemory), "vkAllocateMemory(atlas)");
    CheckVk(vkBindImageMemory(_device, newImage, newMemory, 0), "vkBindImageMemory(atlas)");

    RunOneTimeCommands(_device, _queue, _commandPool, [&](VkCommandBuffer cmd) {
        TransitionImageLayout(
            cmd, newImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            newCapacity);

        if (_atlasImage != VK_NULL_HANDLE && _atlasLayers > 0)
        {
            // Old image is already sitting in GENERAL (used for both transfer + sampling once
            // initialised); transition a copy-source-compatible view of it and copy existing
            // layers across into the new, larger image.
            TransitionImageLayout(
                cmd, _atlasImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                _atlasCapacity);

            VkImageCopy region{};
            region.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, _atlasLayers };
            region.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, _atlasLayers };
            region.extent = { static_cast<uint32_t>(_atlasDimensions), static_cast<uint32_t>(_atlasDimensions), 1 };
            vkCmdCopyImage(
                cmd, _atlasImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, newImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                &region);
        }

        TransitionImageLayout(
            cmd, newImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            newCapacity);
    });

    // The old atlas image's view is bound into each frame-in-flight's descriptor set, and only
    // gets rebound (to whichever atlas is current) once that frame is next recorded. RunOneTimeCommands
    // above only waits for its own one-off submission - it does NOT guarantee that some OTHER
    // frame's previously-submitted (and still executing) command buffer has finished sampling
    // the OLD image via its still-bound descriptor set. Since atlas growth can happen mid-session
    // (whenever a not-yet-cached sprite is first seen - this is common for peeps, which have far
    // more distinct animation/direction sprite variants than e.g. rides, so growth keeps getting
    // triggered long after startup), destroying the old image/view here without waiting for the
    // device to go fully idle first is a use-after-free race that intermittently corrupts
    // sampling results (seen as sprites flickering/changing colour) for whichever frame is still
    // in flight on the GPU at the moment of destruction.
    vkDeviceWaitIdle(_device);

    if (_atlasImageView != VK_NULL_HANDLE)
        vkDestroyImageView(_device, _atlasImageView, nullptr);
    if (_atlasImage != VK_NULL_HANDLE)
        vkDestroyImage(_device, _atlasImage, nullptr);
    if (_atlasImageMemory != VK_NULL_HANDLE)
        vkFreeMemory(_device, _atlasImageMemory, nullptr);

    _atlasImage = newImage;
    _atlasImageMemory = newMemory;
    _atlasCapacity = newCapacity;
    _atlasLayers = newLayerCount;

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = _atlasImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    viewInfo.format = VK_FORMAT_R8_UINT;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = _atlasCapacity;
    CheckVk(vkCreateImageView(_device, &viewInfo, nullptr, &_atlasImageView), "vkCreateImageView(atlas)");
}

void VulkanTextureCache::GeneratePaletteTexture()
{
    static_assert(kPaletteTotalOffsets + 5 < 256, "Height of palette too large!");
    constexpr int32_t width = 256;
    constexpr int32_t height = 256;

    RenderTarget rt = CreateScratchRT(width, height);

    for (int32_t i = 0; i < width; ++i)
    {
        rt.bits[i] = static_cast<PaletteIndex>(i);
    }

    for (int32_t i = 0; i < kPaletteTotalOffsets; ++i)
    {
        auto filterPaletteId = static_cast<FilterPaletteID>(i);
        int32_t y = PaletteToY(filterPaletteId);

        auto g1Index = GetPaletteG1Index(filterPaletteId);
        if (g1Index.has_value())
        {
            const auto* element = GfxGetG1Element(g1Index.value());
            if (element != nullptr)
            {
                GfxDrawSpriteSoftware(rt, ImageId(g1Index.value()), { -element->xOffset, y - element->yOffset });
            }
        }
    }

    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8_UINT;
    imageInfo.extent = { width, height, 1 };
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    CheckVk(vkCreateImage(_device, &imageInfo, nullptr, &_paletteImage), "vkCreateImage(palette)");

    VkMemoryRequirements memReq{};
    vkGetImageMemoryRequirements(_device, _paletteImage, &memReq);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = memReq.size;
    alloc.memoryTypeIndex = FindMemoryType(_physicalDevice, memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    CheckVk(vkAllocateMemory(_device, &alloc, nullptr, &_paletteImageMemory), "vkAllocateMemory(palette)");
    CheckVk(vkBindImageMemory(_device, _paletteImage, _paletteImageMemory, 0), "vkBindImageMemory(palette)");

    VkDeviceSize bufferSize = static_cast<VkDeviceSize>(width) * height;
    VulkanBuffer staging = CreateBuffer(
        _device, _physicalDevice, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    std::memcpy(staging.mapped, rt.bits, bufferSize);

    RunOneTimeCommands(_device, _queue, _commandPool, [&](VkCommandBuffer cmd) {
        TransitionImageLayout(
            cmd, _paletteImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

        VkBufferImageCopy copy{};
        copy.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        copy.imageExtent = { width, height, 1 };
        vkCmdCopyBufferToImage(cmd, staging.buffer, _paletteImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

        TransitionImageLayout(
            cmd, _paletteImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    });

    DestroyBuffer(_device, staging);
    DeleteScratchRT(rt);

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = _paletteImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8_UINT;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;
    CheckVk(vkCreateImageView(_device, &viewInfo, nullptr, &_paletteImageView), "vkCreateImageView(palette)");
}

VulkanAtlasTextureInfo VulkanTextureCache::LoadImageTexture(ImageId imageId)
{
    auto g1Element = GfxGetG1Element(imageId);
    int32_t width = g1Element->width;
    int32_t height = g1Element->height;

    // The atlas is cached/keyed by sprite index only (see GetOrLoadImageTexture), so the pixel
    // data baked in here must NOT have this particular imageId's colour applied - otherwise
    // whichever ImageId (i.e. whichever peep's colours) first triggers the load for a given
    // index "wins" and every other draw that reuses this same cached index (extremely common -
    // e.g. every peep sharing the same walk-cycle frame/direction) would incorrectly show that
    // first caller's colours instead of its own. Colour is applied per-draw-instance at shader
    // time instead (see command.palettes/fPalettes in rect_vk.frag), so strip colour/remap here,
    // matching OpenGL's TextureCache::GetImageAsRT (which uses ImageId(imageId.GetIndex())).
    RenderTarget rt = CreateScratchRT(width, height);
    GfxDrawSpriteSoftware(rt, ImageId(imageId.GetIndex()), { -g1Element->xOffset, -g1Element->yOffset });

    VulkanAtlasTextureInfo cacheInfo = AllocateImage(width, height);
    cacheInfo.image = imageId.GetIndex();

    PendingUpload upload;
    upload.atlasLayer = cacheInfo.index;
    upload.bounds = cacheInfo.bounds;
    upload.pixels.assign(reinterpret_cast<uint8_t*>(rt.bits), reinterpret_cast<uint8_t*>(rt.bits) + width * height);
    _pendingUploads.push_back(std::move(upload));

    DeleteScratchRT(rt);
    return cacheInfo;
}

VulkanAtlasTextureInfo VulkanTextureCache::LoadGlyphTexture(ImageId imageId, const PaletteMap& paletteMap)
{
    auto g1Element = GfxGetG1Element(imageId);
    int32_t width = g1Element->width;
    int32_t height = g1Element->height;

    RenderTarget rt = CreateScratchRT(width, height);
    const auto glyphCoords = ScreenCoordsXY{ -g1Element->xOffset, -g1Element->yOffset };
    GfxDrawSpritePaletteSetSoftware(rt, imageId, glyphCoords, paletteMap);

    VulkanAtlasTextureInfo cacheInfo = AllocateImage(width, height);
    cacheInfo.image = imageId.GetIndex();

    PendingUpload upload;
    upload.atlasLayer = cacheInfo.index;
    upload.bounds = cacheInfo.bounds;
    upload.pixels.assign(reinterpret_cast<uint8_t*>(rt.bits), reinterpret_cast<uint8_t*>(rt.bits) + width * height);
    _pendingUploads.push_back(std::move(upload));

    DeleteScratchRT(rt);
    return cacheInfo;
}

VulkanAtlasTextureInfo VulkanTextureCache::LoadBitmapTexture(ImageIndex image, const void* pixels, size_t width, size_t height)
{
    VulkanAtlasTextureInfo cacheInfo = AllocateImage(static_cast<int32_t>(width), static_cast<int32_t>(height));
    cacheInfo.image = image;

    PendingUpload upload;
    upload.atlasLayer = cacheInfo.index;
    upload.bounds = cacheInfo.bounds;
    const auto* bytes = reinterpret_cast<const uint8_t*>(pixels);
    upload.pixels.assign(bytes, bytes + width * height);
    _pendingUploads.push_back(std::move(upload));

    return cacheInfo;
}

void VulkanTextureCache::FlushPendingUploads(VkCommandBuffer cmd, VulkanBuffer& stagingBuffer)
{
    if (_pendingUploads.empty())
        return;

    VkDeviceSize totalSize = 0;
    for (const auto& upload : _pendingUploads)
    {
        totalSize += upload.pixels.size();
    }

    if (stagingBuffer.buffer == VK_NULL_HANDLE || stagingBuffer.size < totalSize)
    {
        DestroyBuffer(_device, stagingBuffer);
        VkDeviceSize newSize = std::max<VkDeviceSize>(totalSize, 1 << 16);
        stagingBuffer = CreateBuffer(
            _device, _physicalDevice, newSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }

    // First frame ever: the atlas image doesn't exist as a "previous" resource in GENERAL yet
    // (EnlargeAtlasesImage already left it in GENERAL after creation/growth), so no extra
    // transition is required here - GENERAL is valid for both vkCmdCopyBufferToImage and
    // subsequent shader sampling.
    VkDeviceSize offset = 0;
    auto* mappedBytes = static_cast<uint8_t*>(stagingBuffer.mapped);
    for (const auto& upload : _pendingUploads)
    {
        std::memcpy(mappedBytes + offset, upload.pixels.data(), upload.pixels.size());

        int32_t width = upload.bounds.z - upload.bounds.x;
        int32_t height = upload.bounds.w - upload.bounds.y;

        VkBufferImageCopy copy{};
        copy.bufferOffset = offset;
        copy.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, static_cast<uint32_t>(upload.atlasLayer), 1 };
        copy.imageOffset = { upload.bounds.x, upload.bounds.y, 0 };
        copy.imageExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
        vkCmdCopyBufferToImage(cmd, stagingBuffer.buffer, _atlasImage, VK_IMAGE_LAYOUT_GENERAL, 1, &copy);

        offset += upload.pixels.size();
    }

    _pendingUploads.clear();
}

int32_t VulkanTextureCache::PaletteToY(FilterPaletteID palette)
{
    return palette > FilterPaletteID::paletteWater ? EnumValue(palette) + 5 : EnumValue(palette) + 1;
}

#endif
