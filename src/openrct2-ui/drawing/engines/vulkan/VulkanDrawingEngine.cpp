/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#ifndef DISABLE_VULKAN

#include "../DrawingEngineFactory.hpp"
#include "VulkanDrawingContext.h"
#include "VulkanHelpers.h"

#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <openrct2-ui/interface/Window.h>
#include <openrct2/Context.h>
#include <openrct2/Diagnostic.h>
#include <openrct2/PlatformEnvironment.h>
#include <openrct2/core/EnumUtils.hpp>
#include <openrct2/core/FileStream.h>
#include <openrct2/core/Guard.hpp>
#include <openrct2/core/Path.hpp>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/IDrawingEngine.h>
#include <openrct2/drawing/InvalidationGrid.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/WeatherDrawer.h>
#include <openrct2/interface/Screenshot.h>
#include <openrct2/ui/UiContext.h>
#include <openrct2/world/Weather.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

namespace
{
    constexpr uint32_t kFramesInFlight = kVulkanFramesInFlight;
} // namespace

class VulkanWeatherDrawer final : public IWeatherDrawer
{
    VulkanDrawingContext* _drawingContext;

public:
    explicit VulkanWeatherDrawer(VulkanDrawingContext* drawingContext)
        : _drawingContext(drawingContext)
    {
    }

    void Draw(
        RenderTarget& rt, int32_t x, int32_t y, int32_t width, int32_t height, int32_t xStart, int32_t yStart,
        const uint8_t* weatherpattern) override
    {
        const uint8_t* pattern = weatherpattern;
        auto patternXSpace = *pattern++;
        auto patternYSpace = *pattern++;

        uint8_t patternStartXOffset = xStart % patternXSpace;
        uint8_t patternStartYOffset = yStart % patternYSpace;

        uint32_t pixelOffset = rt.LineStride() * y + x;
        uint8_t patternYPos = patternStartYOffset % patternYSpace;

        for (; height != 0; height--)
        {
            auto patternX = pattern[patternYPos * 2];
            if (patternX != 0xFF)
            {
                uint32_t finalPixelOffset = width + pixelOffset;

                uint32_t xPixelOffset = pixelOffset;
                xPixelOffset += (static_cast<uint8_t>(patternX - patternStartXOffset)) % patternXSpace;

                auto patternPixel = static_cast<PaletteIndex>(pattern[patternYPos * 2 + 1]);
                for (; xPixelOffset < finalPixelOffset; xPixelOffset += patternXSpace)
                {
                    int32_t pixelX = xPixelOffset % rt.width;
                    int32_t pixelY = (xPixelOffset / rt.width) % rt.height;

                    _drawingContext->DrawLine(rt, patternPixel, { { pixelX, pixelY }, { pixelX + 1, pixelY + 1 } });
                }
            }

            pixelOffset += rt.LineStride();
            patternYPos++;
            patternYPos %= patternYSpace;
        }
    }
};

class VulkanDrawingEngine final : public IDrawingEngine
{
private:
    IUiContext& _uiContext;
    SDL_Window* _window = nullptr;

    VkInstance _instance = VK_NULL_HANDLE;
    VkSurfaceKHR _surface = VK_NULL_HANDLE;
    VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
    VkDevice _device = VK_NULL_HANDLE;
    uint32_t _graphicsQueueFamily = std::numeric_limits<uint32_t>::max();
    VkQueue _graphicsQueue = VK_NULL_HANDLE;

    VkSwapchainKHR _swapchain = VK_NULL_HANDLE;
    VkFormat _swapchainFormat = VK_FORMAT_B8G8R8A8_UNORM;
    VkExtent2D _swapchainExtent{};
    std::vector<VkImage> _swapchainImages;

    VkCommandPool _commandPool = VK_NULL_HANDLE;
    std::array<VkCommandBuffer, kFramesInFlight> _commandBuffers{};

    std::array<VkSemaphore, kFramesInFlight> _imageAvailable{};
    std::array<VkSemaphore, kFramesInFlight> _renderFinished{};
    std::array<VkFence, kFramesInFlight> _inFlightFences{};
    uint32_t _currentFrame = 0;

    std::vector<VkImageView> _swapchainImageViews;
    std::vector<VkFramebuffer> _framebuffers;

    // Device-lifetime shader/pipeline resources for the final composite pass (palette lookup
    // from the drawing context's R8_UINT offscreen colour target into the presentable swapchain
    // image) - created once and reused across swapchain recreations.
    VkRenderPass _renderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout _descriptorSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkSampler _paletteIndexSampler = VK_NULL_HANDLE;
    VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, kFramesInFlight> _descriptorSets{};

    // Palette colour lookup table, uploaded to a uniform buffer read by the composite fragment
    // shader. One per frame-in-flight so an update while a previous frame is still in flight
    // can't race.
    std::array<VkBuffer, kFramesInFlight> _paletteBuffers{};
    std::array<VkDeviceMemory, kFramesInFlight> _paletteBufferMemories{};
    std::array<void*, kFramesInFlight> _paletteBuffersMapped{};
    std::array<float, 256 * 4> _paletteData{};

    // Scratch image used only by CopyRect() to move a rectangular region within the drawing
    // context's persistent colour image (window/list scrolling) - sized to the full render
    // resolution and recreated on resize.
    VkImage _copyTempImage = VK_NULL_HANDLE;
    VkDeviceMemory _copyTempImageMemory = VK_NULL_HANDLE;

    VulkanDrawingContext _drawingContext;
    VulkanWeatherDrawer _weatherDrawer;
    InvalidationGrid _invalidationGrid;

    uint32_t _width = 0;
    uint32_t _height = 0;
    uint32_t _pitch = 0;
    size_t _bitsSize = 0;
    std::unique_ptr<PaletteIndex[]> _bits;
    RenderTarget _mainRT{};

    bool _useVsync = true;

public:
    explicit VulkanDrawingEngine(IUiContext& uiContext)
        : _uiContext(uiContext)
        , _weatherDrawer(&_drawingContext)
    {
        _window = static_cast<SDL_Window*>(_uiContext.GetWindow());
        _mainRT.DrawingEngine = this;
    }

    ~VulkanDrawingEngine() override
    {
        DestroyVulkan();
    }

    void Initialise() override
    {
        CreateInstance();
        CreateSurface();
        PickPhysicalDeviceAndQueue();
        CreateDevice();
        CreateCommandPool();
        CreateSyncObjects();
        CreateDescriptorSetLayout();
        CreatePipelineLayout();
        CreateSampler();
        CreateDescriptorPool();
        CreatePaletteBuffers();

        _drawingContext.Initialise(_device, _physicalDevice, _graphicsQueue, _commandPool, &_mainRT);

        LOG_VERBOSE("Vulkan renderer initialised.");
    }

    void Resize(uint32_t width, uint32_t height) override
    {
        ConfigureBits(width, height, width);
        ConfigureDirtyGrid();
        RecreateSwapchainAndResources();
    }

    void ConfigureDirtyGrid()
    {
        const auto blockWidth = 1u << 8;
        const auto blockHeight = 1u << 8;
        _invalidationGrid.reset(_width, _height, blockWidth, blockHeight);
    }

    // Maintains a dummy CPU-side pixel buffer that is never actually rendered into - its only
    // purpose is so RenderTarget::Crop()'d sub-render-targets can be mapped back to an absolute
    // screen-space rectangle via pointer arithmetic (see VulkanDrawingContext::CalculateClipping),
    // exactly mirroring OpenGLDrawingEngine::ConfigureBits.
    void ConfigureBits(uint32_t width, uint32_t height, uint32_t pitch)
    {
        size_t newBitsSize = static_cast<size_t>(pitch) * height;

        auto newBits = std::make_unique<PaletteIndex[]>(newBitsSize);
        if (_bits == nullptr)
        {
            std::fill_n(newBits.get(), newBitsSize, PaletteIndex::transparent);
        }
        else
        {
            if (_pitch == pitch)
            {
                std::copy_n(_bits.get(), std::min(_bitsSize, newBitsSize), newBits.get());
            }
            else
            {
                PaletteIndex* src = _bits.get();
                PaletteIndex* dst = newBits.get();

                uint32_t minWidth = std::min(_width, width);
                uint32_t minHeight = std::min(_height, height);
                for (uint32_t y = 0; y < minHeight; y++)
                {
                    std::copy_n(src, minWidth, dst);
                    if (pitch - minWidth > 0)
                    {
                        std::fill_n(dst + minWidth, pitch - minWidth, PaletteIndex::transparent);
                    }
                    src += _pitch;
                    dst += pitch;
                }
            }
        }

        _bits = std::move(newBits);
        _bitsSize = newBitsSize;
        _width = width;
        _height = height;
        _pitch = pitch;

        RenderTarget* rt = &_mainRT;
        rt->bits = _bits.get();
        rt->x = 0;
        rt->y = 0;
        rt->width = width;
        rt->height = height;
        rt->pitch = _pitch - width;
    }

    void SetPalette(const GamePalette& palette) override
    {
        for (int32_t i = 0; i < 256; i++)
        {
            const float a = (i == 0) ? 0.0f : 1.0f;
            _paletteData[static_cast<size_t>(i) * 4 + 0] = palette[i].red / 255.0f;
            _paletteData[static_cast<size_t>(i) * 4 + 1] = palette[i].green / 255.0f;
            _paletteData[static_cast<size_t>(i) * 4 + 2] = palette[i].blue / 255.0f;
            _paletteData[static_cast<size_t>(i) * 4 + 3] = a;
        }

        if (_device == VK_NULL_HANDLE)
            return;

        // Palette changes are rare (not a per-frame hot path), so it's fine to wait for the
        // device to go idle here rather than adding extra complexity to update every
        // frame-in-flight's uniform buffer copy without racing a frame that might still be
        // reading the old contents on the GPU.
        vkDeviceWaitIdle(_device);
        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            if (_paletteBuffersMapped[i] != nullptr)
            {
                std::memcpy(_paletteBuffersMapped[i], _paletteData.data(), _paletteData.size() * sizeof(float));
            }
        }
    }

    void SetVSync(bool vsync) override
    {
        if (_useVsync != vsync)
        {
            _useVsync = vsync;
            RecreateSwapchainAndResources();
        }
    }

    void Invalidate(int32_t left, int32_t top, int32_t right, int32_t bottom) override
    {
        _invalidationGrid.invalidate(left, top, right, bottom);
    }

    void BeginDraw() override
    {
        _drawingContext.StartNewDraw();
    }

    void EndDraw() override
    {
        Present();
        _drawingContext.FinishDraw();
    }

    void PaintWindows() override
    {
        if (Weather::hasWeatherEffect() || gPaintForceRedraw)
        {
            WindowUpdateAllViewports();
            // No support for restoring pixels between frames (the offscreen colour target is
            // persistent, but a moving weather overlay would otherwise leave old raindrops
            // behind), so always redraw the whole screen while weather is active.
            WindowDrawAll(_mainRT, 0, 0, static_cast<int32_t>(_width), static_cast<int32_t>(_height));
        }
        else
        {
            // Redraw dirty regions before updating the viewports, otherwise when viewports get
            // panned, they copy dirty pixels.
            DrawAllDirtyBlocks();
            WindowUpdateAllViewports();
            DrawAllDirtyBlocks();
        }
    }

    void DrawAllDirtyBlocks()
    {
        _invalidationGrid.traverseDirtyCells([this](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            WindowDrawAll(_mainRT, left, top, right, bottom);
        });
    }

    void PaintWeather() override
    {
        DrawWeather(_mainRT, &_weatherDrawer);
    }

    std::string Screenshot() override
    {
        // Not yet implemented for the Vulkan renderer's new offscreen-GPU architecture - reading
        // back the colour attachment into _mainRT and dumping it as a PNG needs a staging buffer
        // + vkCmdCopyImageToBuffer round trip. Fall back to an empty result rather than crashing.
        return {};
    }

    void CopyRect(int32_t x, int32_t y, int32_t width, int32_t height, int32_t dx, int32_t dy) override
    {
        if (dx == 0 && dy == 0)
            return;
        if (_device == VK_NULL_HANDLE || _copyTempImage == VK_NULL_HANDLE)
            return;

        const int32_t texWidth = static_cast<int32_t>(_width);
        const int32_t texHeight = static_cast<int32_t>(_height);

        // Adjust for move off screen
        int32_t lmargin = std::min(x - dx, 0);
        int32_t rmargin = std::min(texWidth - (x - dx + width), 0);
        int32_t tmargin = std::min(y - dy, 0);
        int32_t bmargin = std::min(texHeight - (y - dy + height), 0);
        x -= lmargin;
        y -= tmargin;
        width += lmargin + rmargin;
        height += tmargin + bmargin;

        if (width <= 0 || height <= 0)
            return;

        // Flush any draw calls queued so far this frame into the persistent colour image before
        // copying from it, matching OpenGLDrawingEngine::CopyRect's FlushCommandBuffers() call.
        RunOneTimeCommands([this](VkCommandBuffer cmd) { _drawingContext.FlushCommandBuffers(cmd, _currentFrame); });

        // Both the drawing context's colour image and this scratch image are kept permanently
        // in VK_IMAGE_LAYOUT_GENERAL (a valid layout for vkCmdCopyImage source/destination), so
        // no layout transitions are needed - just two copies through the scratch image (a direct
        // same-image overlapping-region copy is not portably expressible in Vulkan).
        RunOneTimeCommands([&](VkCommandBuffer cmd) {
            VkImageCopy toTemp{};
            toTemp.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
            toTemp.srcOffset = { x - dx, y - dy, 0 };
            toTemp.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
            toTemp.dstOffset = { 0, 0, 0 };
            toTemp.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
            vkCmdCopyImage(
                cmd, _drawingContext.GetColourImage(), VK_IMAGE_LAYOUT_GENERAL, _copyTempImage, VK_IMAGE_LAYOUT_GENERAL, 1,
                &toTemp);

            VkMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            vkCmdPipelineBarrier(
                cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &barrier, 0, nullptr, 0, nullptr);

            VkImageCopy toMain{};
            toMain.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
            toMain.srcOffset = { 0, 0, 0 };
            toMain.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
            toMain.dstOffset = { x, y, 0 };
            toMain.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
            vkCmdCopyImage(
                cmd, _copyTempImage, VK_IMAGE_LAYOUT_GENERAL, _drawingContext.GetColourImage(), VK_IMAGE_LAYOUT_GENERAL, 1,
                &toMain);
        });
    }

    IDrawingContext* GetDrawingContext() override
    {
        if (!_drawingContext.IsActive())
        {
            Guard::Fail("Drawing context is not active.");
            return nullptr;
        }
        return &_drawingContext;
    }

    RenderTarget* getRT() override
    {
        return &_mainRT;
    }

    DrawingEngineFlags GetFlags() override
    {
        return DrawingEngineFlag::dirtyOptimisations;
    }

    void InvalidateImage(uint32_t image) override
    {
        _drawingContext.GetTextureCache().InvalidateImage(image);
    }

private:
    void DestroyVulkan()
    {
        if (_device != VK_NULL_HANDLE)
        {
            vkDeviceWaitIdle(_device);
        }

        DestroySwapchainAndResources();
        _drawingContext.Destroy();

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            if (_paletteBuffers[i] != VK_NULL_HANDLE)
            {
                if (_paletteBuffersMapped[i] != nullptr)
                {
                    vkUnmapMemory(_device, _paletteBufferMemories[i]);
                    _paletteBuffersMapped[i] = nullptr;
                }
                vkDestroyBuffer(_device, _paletteBuffers[i], nullptr);
            }
            if (_paletteBufferMemories[i] != VK_NULL_HANDLE)
                vkFreeMemory(_device, _paletteBufferMemories[i], nullptr);
        }

        if (_descriptorPool != VK_NULL_HANDLE)
            vkDestroyDescriptorPool(_device, _descriptorPool, nullptr);
        if (_pipeline != VK_NULL_HANDLE)
            vkDestroyPipeline(_device, _pipeline, nullptr);
        if (_pipelineLayout != VK_NULL_HANDLE)
            vkDestroyPipelineLayout(_device, _pipelineLayout, nullptr);
        if (_renderPass != VK_NULL_HANDLE)
            vkDestroyRenderPass(_device, _renderPass, nullptr);
        if (_paletteIndexSampler != VK_NULL_HANDLE)
            vkDestroySampler(_device, _paletteIndexSampler, nullptr);
        if (_descriptorSetLayout != VK_NULL_HANDLE)
            vkDestroyDescriptorSetLayout(_device, _descriptorSetLayout, nullptr);

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            if (_imageAvailable[i] != VK_NULL_HANDLE)
                vkDestroySemaphore(_device, _imageAvailable[i], nullptr);
            if (_renderFinished[i] != VK_NULL_HANDLE)
                vkDestroySemaphore(_device, _renderFinished[i], nullptr);
            if (_inFlightFences[i] != VK_NULL_HANDLE)
                vkDestroyFence(_device, _inFlightFences[i], nullptr);
        }

        if (_commandPool != VK_NULL_HANDLE)
            vkDestroyCommandPool(_device, _commandPool, nullptr);
        if (_device != VK_NULL_HANDLE)
            vkDestroyDevice(_device, nullptr);
        if (_surface != VK_NULL_HANDLE)
            vkDestroySurfaceKHR(_instance, _surface, nullptr);
        if (_instance != VK_NULL_HANDLE)
            vkDestroyInstance(_instance, nullptr);
    }

    void CreateInstance()
    {
        uint32_t extCount = 0;
        if (SDL_Vulkan_GetInstanceExtensions(_window, &extCount, nullptr) != SDL_TRUE)
        {
            throw std::runtime_error(std::string("SDL_Vulkan_GetInstanceExtensions failed: ") + SDL_GetError());
        }

        std::vector<const char*> extensions(extCount);
        if (SDL_Vulkan_GetInstanceExtensions(_window, &extCount, extensions.data()) != SDL_TRUE)
        {
            throw std::runtime_error(std::string("SDL_Vulkan_GetInstanceExtensions failed: ") + SDL_GetError());
        }

        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "OpenRCT2";
        appInfo.pEngineName = "OpenRCT2";
        appInfo.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount = extCount;
        createInfo.ppEnabledExtensionNames = extensions.data();

        CheckVk(vkCreateInstance(&createInfo, nullptr, &_instance), "vkCreateInstance");
    }

    void CreateSurface()
    {
        if (SDL_Vulkan_CreateSurface(_window, _instance, &_surface) != SDL_TRUE)
        {
            throw std::runtime_error(std::string("SDL_Vulkan_CreateSurface failed: ") + SDL_GetError());
        }
    }

    void PickPhysicalDeviceAndQueue()
    {
        uint32_t count = 0;
        CheckVk(vkEnumeratePhysicalDevices(_instance, &count, nullptr), "vkEnumeratePhysicalDevices(count)");
        if (count == 0)
        {
            throw std::runtime_error("No Vulkan physical devices found.");
        }

        std::vector<VkPhysicalDevice> devices(count);
        CheckVk(vkEnumeratePhysicalDevices(_instance, &count, devices.data()), "vkEnumeratePhysicalDevices(list)");

        for (auto device : devices)
        {
            uint32_t queueFamilyCount = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
            std::vector<VkQueueFamilyProperties> queueFamilyProps(queueFamilyCount);
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilyProps.data());

            for (uint32_t i = 0; i < queueFamilyCount; i++)
            {
                VkBool32 presentSupport = VK_FALSE;
                CheckVk(
                    vkGetPhysicalDeviceSurfaceSupportKHR(device, i, _surface, &presentSupport),
                    "vkGetPhysicalDeviceSurfaceSupportKHR");

                const bool graphics = (queueFamilyProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;
                if (graphics && presentSupport == VK_TRUE)
                {
                    _physicalDevice = device;
                    _graphicsQueueFamily = i;

                    VkPhysicalDeviceProperties gpuProps{};
                    vkGetPhysicalDeviceProperties(_physicalDevice, &gpuProps);
                    LOG_VERBOSE(
                        "Vulkan physical device selected: %s (API %u.%u.%u)", gpuProps.deviceName,
                        VK_VERSION_MAJOR(gpuProps.apiVersion), VK_VERSION_MINOR(gpuProps.apiVersion),
                        VK_VERSION_PATCH(gpuProps.apiVersion));
                    return;
                }
            }
        }

        throw std::runtime_error("No Vulkan queue family supports both graphics and present.");
    }

    void CreateDevice()
    {
        float priority = 1.0f;
        VkDeviceQueueCreateInfo queueInfo{};
        queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueInfo.queueFamilyIndex = _graphicsQueueFamily;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &priority;

        const char* extensions[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };

        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.queueCreateInfoCount = 1;
        createInfo.pQueueCreateInfos = &queueInfo;
        createInfo.enabledExtensionCount = 1;
        createInfo.ppEnabledExtensionNames = extensions;

        CheckVk(vkCreateDevice(_physicalDevice, &createInfo, nullptr, &_device), "vkCreateDevice");
        vkGetDeviceQueue(_device, _graphicsQueueFamily, 0, &_graphicsQueue);
    }

    void CreateCommandPool()
    {
        VkCommandPoolCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        info.queueFamilyIndex = _graphicsQueueFamily;
        info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        CheckVk(vkCreateCommandPool(_device, &info, nullptr, &_commandPool), "vkCreateCommandPool");
    }

    void CreateSyncObjects()
    {
        VkSemaphoreCreateInfo semInfo{};
        semInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            CheckVk(vkCreateSemaphore(_device, &semInfo, nullptr, &_imageAvailable[i]), "vkCreateSemaphore(imageAvailable)");
            CheckVk(vkCreateSemaphore(_device, &semInfo, nullptr, &_renderFinished[i]), "vkCreateSemaphore(renderFinished)");
            CheckVk(vkCreateFence(_device, &fenceInfo, nullptr, &_inFlightFences[i]), "vkCreateFence");
        }
    }

    void DestroySwapchainAndResources()
    {
        if (_device == VK_NULL_HANDLE)
            return;

        if (_copyTempImage != VK_NULL_HANDLE)
        {
            vkDestroyImage(_device, _copyTempImage, nullptr);
            _copyTempImage = VK_NULL_HANDLE;
        }
        if (_copyTempImageMemory != VK_NULL_HANDLE)
        {
            vkFreeMemory(_device, _copyTempImageMemory, nullptr);
            _copyTempImageMemory = VK_NULL_HANDLE;
        }

        for (auto& cmd : _commandBuffers)
        {
            if (cmd != VK_NULL_HANDLE)
            {
                vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);
                cmd = VK_NULL_HANDLE;
            }
        }

        for (auto framebuffer : _framebuffers)
        {
            vkDestroyFramebuffer(_device, framebuffer, nullptr);
        }
        _framebuffers.clear();

        for (auto imageView : _swapchainImageViews)
        {
            vkDestroyImageView(_device, imageView, nullptr);
        }
        _swapchainImageViews.clear();

        if (_swapchain != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(_device, _swapchain, nullptr);
            _swapchain = VK_NULL_HANDLE;
        }
        _swapchainImages.clear();
    }

    void RecreateSwapchainAndResources()
    {
        if (_device == VK_NULL_HANDLE || _width == 0 || _height == 0)
            return;

        vkDeviceWaitIdle(_device);
        DestroySwapchainAndResources();
        CreateSwapchain();
        CreateSwapchainImageViews();
        if (_renderPass == VK_NULL_HANDLE)
        {
            // The render pass and pipeline depend on the swapchain's surface format, which is
            // only known once the swapchain has been created for the first time. They don't
            // depend on its extent though, so they only need to be created once, ever.
            CreateRenderPass();
            CreateGraphicsPipeline();
        }
        CreateFramebuffers();
        CreateCopyTempImage();
        CreateCommandBuffers();

        _drawingContext.Resize(_width, _height);
        UpdateCompositeDescriptorSets();
    }

    // Runs a single one-shot command buffer synchronously. Not used in the regular per-frame
    // hot path (Present() records/submits its own per-frame command buffer) - only for
    // (re)creation of swapchain/image resources and for CopyRect's occasional GPU-side copies.
    void RunOneTimeCommands(const std::function<void(VkCommandBuffer)>& fn)
    {
        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = _commandPool;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = 1;

        VkCommandBuffer cmd = VK_NULL_HANDLE;
        CheckVk(vkAllocateCommandBuffers(_device, &alloc, &cmd), "vkAllocateCommandBuffers(one-time)");

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        CheckVk(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer(one-time)");

        fn(cmd);

        CheckVk(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(one-time)");

        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        CheckVk(vkQueueSubmit(_graphicsQueue, 1, &submit, VK_NULL_HANDLE), "vkQueueSubmit(one-time)");
        CheckVk(vkQueueWaitIdle(_graphicsQueue), "vkQueueWaitIdle(one-time)");

        vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);
    }

    void CreateSwapchain()
    {
        VkSurfaceCapabilitiesKHR caps{};
        CheckVk(
            vkGetPhysicalDeviceSurfaceCapabilitiesKHR(_physicalDevice, _surface, &caps),
            "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");

        uint32_t formatCount = 0;
        CheckVk(
            vkGetPhysicalDeviceSurfaceFormatsKHR(_physicalDevice, _surface, &formatCount, nullptr),
            "vkGetPhysicalDeviceSurfaceFormatsKHR(count)");
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        CheckVk(
            vkGetPhysicalDeviceSurfaceFormatsKHR(_physicalDevice, _surface, &formatCount, formats.data()),
            "vkGetPhysicalDeviceSurfaceFormatsKHR(list)");

        VkSurfaceFormatKHR chosenFormat = formats[0];
        for (const auto& f : formats)
        {
            if (f.format == VK_FORMAT_B8G8R8A8_UNORM && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                chosenFormat = f;
                break;
            }
        }
        _swapchainFormat = chosenFormat.format;

        uint32_t presentModeCount = 0;
        CheckVk(
            vkGetPhysicalDeviceSurfacePresentModesKHR(_physicalDevice, _surface, &presentModeCount, nullptr),
            "vkGetPhysicalDeviceSurfacePresentModesKHR(count)");
        std::vector<VkPresentModeKHR> presentModes(presentModeCount);
        CheckVk(
            vkGetPhysicalDeviceSurfacePresentModesKHR(_physicalDevice, _surface, &presentModeCount, presentModes.data()),
            "vkGetPhysicalDeviceSurfacePresentModesKHR(list)");

        VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
        if (!_useVsync)
        {
            for (auto mode : presentModes)
            {
                if (mode == VK_PRESENT_MODE_MAILBOX_KHR)
                {
                    presentMode = mode;
                    break;
                }
            }

            if (presentMode == VK_PRESENT_MODE_FIFO_KHR)
            {
                for (auto mode : presentModes)
                {
                    if (mode == VK_PRESENT_MODE_IMMEDIATE_KHR)
                    {
                        presentMode = mode;
                        break;
                    }
                }
            }
        }

        _swapchainExtent.width = _width;
        _swapchainExtent.height = _height;
        if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max())
        {
            _swapchainExtent = caps.currentExtent;
        }

        uint32_t imageCount = caps.minImageCount + 1;
        if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
            imageCount = caps.maxImageCount;

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = _surface;
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = chosenFormat.format;
        createInfo.imageColorSpace = chosenFormat.colorSpace;
        createInfo.imageExtent = _swapchainExtent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        createInfo.preTransform = caps.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;
        createInfo.oldSwapchain = VK_NULL_HANDLE;

        CheckVk(vkCreateSwapchainKHR(_device, &createInfo, nullptr, &_swapchain), "vkCreateSwapchainKHR");

        uint32_t swapCount = 0;
        CheckVk(vkGetSwapchainImagesKHR(_device, _swapchain, &swapCount, nullptr), "vkGetSwapchainImagesKHR(count)");
        _swapchainImages.resize(swapCount);
        CheckVk(
            vkGetSwapchainImagesKHR(_device, _swapchain, &swapCount, _swapchainImages.data()),
            "vkGetSwapchainImagesKHR(list)");
    }

    void CreateSwapchainImageViews()
    {
        _swapchainImageViews.resize(_swapchainImages.size());
        for (size_t i = 0; i < _swapchainImages.size(); i++)
        {
            VkImageViewCreateInfo info{};
            info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            info.image = _swapchainImages[i];
            info.viewType = VK_IMAGE_VIEW_TYPE_2D;
            info.format = _swapchainFormat;
            info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            info.subresourceRange.levelCount = 1;
            info.subresourceRange.layerCount = 1;
            CheckVk(vkCreateImageView(_device, &info, nullptr, &_swapchainImageViews[i]), "vkCreateImageView(swapchain)");
        }
    }

    // Render pass used to draw the fullscreen palette-lookup triangle into a swapchain image.
    // Its initial/final layouts mean Vulkan handles the UNDEFINED -> COLOR_ATTACHMENT_OPTIMAL
    // -> PRESENT_SRC_KHR transitions for us automatically, so no manual barriers are needed
    // around it.
    void CreateRenderPass()
    {
        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = _swapchainFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorRef{};
        colorRef.attachment = 0;
        colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;

        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.srcAccessMask = 0;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        info.attachmentCount = 1;
        info.pAttachments = &colorAttachment;
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = 1;
        info.pDependencies = &dependency;

        CheckVk(vkCreateRenderPass(_device, &info, nullptr, &_renderPass), "vkCreateRenderPass");
    }

    void CreateFramebuffers()
    {
        _framebuffers.resize(_swapchainImageViews.size());
        for (size_t i = 0; i < _swapchainImageViews.size(); i++)
        {
            VkImageView attachments[] = { _swapchainImageViews[i] };

            VkFramebufferCreateInfo info{};
            info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            info.renderPass = _renderPass;
            info.attachmentCount = 1;
            info.pAttachments = attachments;
            info.width = _swapchainExtent.width;
            info.height = _swapchainExtent.height;
            info.layers = 1;
            CheckVk(vkCreateFramebuffer(_device, &info, nullptr, &_framebuffers[i]), "vkCreateFramebuffer");
        }
    }

    void CreateCopyTempImage()
    {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = VK_FORMAT_R8_UINT;
        imageInfo.extent = { _width, _height, 1 };
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        CheckVk(vkCreateImage(_device, &imageInfo, nullptr, &_copyTempImage), "vkCreateImage(copyTemp)");

        VkMemoryRequirements memReq{};
        vkGetImageMemoryRequirements(_device, _copyTempImage, &memReq);
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReq.size;
        alloc.memoryTypeIndex = FindMemoryType(_physicalDevice, memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        CheckVk(vkAllocateMemory(_device, &alloc, nullptr, &_copyTempImageMemory), "vkAllocateMemory(copyTemp)");
        CheckVk(vkBindImageMemory(_device, _copyTempImage, _copyTempImageMemory, 0), "vkBindImageMemory(copyTemp)");

        RunOneTimeCommands([this](VkCommandBuffer cmd) {
            TransitionImageLayout(
                cmd, _copyTempImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
        });
    }

    void CreateDescriptorSetLayout()
    {
        VkDescriptorSetLayoutBinding samplerBinding{};
        samplerBinding.binding = 0;
        samplerBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        samplerBinding.descriptorCount = 1;
        samplerBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutBinding uboBinding{};
        uboBinding.binding = 1;
        uboBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uboBinding.descriptorCount = 1;
        uboBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutBinding bindings[] = { samplerBinding, uboBinding };

        VkDescriptorSetLayoutCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        info.bindingCount = 2;
        info.pBindings = bindings;
        CheckVk(vkCreateDescriptorSetLayout(_device, &info, nullptr, &_descriptorSetLayout), "vkCreateDescriptorSetLayout");
    }

    void CreatePipelineLayout()
    {
        VkPipelineLayoutCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        info.setLayoutCount = 1;
        info.pSetLayouts = &_descriptorSetLayout;
        CheckVk(vkCreatePipelineLayout(_device, &info, nullptr, &_pipelineLayout), "vkCreatePipelineLayout");
    }

    void CreateSampler()
    {
        VkSamplerCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        info.magFilter = VK_FILTER_NEAREST;
        info.minFilter = VK_FILTER_NEAREST;
        info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        CheckVk(vkCreateSampler(_device, &info, nullptr, &_paletteIndexSampler), "vkCreateSampler");
    }

    void CreateDescriptorPool()
    {
        VkDescriptorPoolSize sizes[2]{};
        sizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        sizes[0].descriptorCount = kFramesInFlight;
        sizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        sizes[1].descriptorCount = kFramesInFlight;

        VkDescriptorPoolCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        info.maxSets = kFramesInFlight;
        info.poolSizeCount = 2;
        info.pPoolSizes = sizes;
        CheckVk(vkCreateDescriptorPool(_device, &info, nullptr, &_descriptorPool), "vkCreateDescriptorPool");

        std::array<VkDescriptorSetLayout, kFramesInFlight> layouts{};
        layouts.fill(_descriptorSetLayout);

        VkDescriptorSetAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        alloc.descriptorPool = _descriptorPool;
        alloc.descriptorSetCount = kFramesInFlight;
        alloc.pSetLayouts = layouts.data();
        CheckVk(vkAllocateDescriptorSets(_device, &alloc, _descriptorSets.data()), "vkAllocateDescriptorSets");
    }

    void CreatePaletteBuffers()
    {
        const VkDeviceSize bufferSize = _paletteData.size() * sizeof(float);
        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            VkBufferCreateInfo info{};
            info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            info.size = bufferSize;
            info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            CheckVk(vkCreateBuffer(_device, &info, nullptr, &_paletteBuffers[i]), "vkCreateBuffer(palette)");

            VkMemoryRequirements memReq{};
            vkGetBufferMemoryRequirements(_device, _paletteBuffers[i], &memReq);

            VkMemoryAllocateInfo alloc{};
            alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            alloc.allocationSize = memReq.size;
            alloc.memoryTypeIndex = FindMemoryType(
                _physicalDevice, memReq.memoryTypeBits,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            CheckVk(vkAllocateMemory(_device, &alloc, nullptr, &_paletteBufferMemories[i]), "vkAllocateMemory(palette)");
            CheckVk(
                vkBindBufferMemory(_device, _paletteBuffers[i], _paletteBufferMemories[i], 0), "vkBindBufferMemory(palette)");
            CheckVk(
                vkMapMemory(_device, _paletteBufferMemories[i], 0, bufferSize, 0, &_paletteBuffersMapped[i]),
                "vkMapMemory(palette)");

            // Initialise with whatever palette data we currently have (may just be the
            // zero-initialised default if SetPalette() hasn't been called yet).
            std::memcpy(_paletteBuffersMapped[i], _paletteData.data(), bufferSize);
        }
    }

    // Binds a single frame-in-flight's descriptor set to the drawing context's CURRENT offscreen
    // colour image view and its own palette uniform buffer. The image view must be refreshed
    // every frame (not just after a resize) because it is one of a ping-ponged pair whose
    // "current" index changes whenever the depth-peeling transparency compositor runs (see
    // VulkanDrawingContext::HandleTransparency / GetColourImageView()).
    void UpdateCompositeDescriptorSet(uint32_t frameIndex)
    {
        VkDescriptorImageInfo imgInfo{};
        imgInfo.sampler = _paletteIndexSampler;
        imgInfo.imageView = _drawingContext.GetColourImageView();
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkDescriptorBufferInfo bufInfo{};
        bufInfo.buffer = _paletteBuffers[frameIndex];
        bufInfo.offset = 0;
        bufInfo.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = _descriptorSets[frameIndex];
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].pImageInfo = &imgInfo;

        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = _descriptorSets[frameIndex];
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[1].pBufferInfo = &bufInfo;

        vkUpdateDescriptorSets(_device, 2, writes, 0, nullptr);
    }

    // Called once after every _drawingContext.Resize() to (re)bind every frame-in-flight's
    // descriptor set, since the underlying images/views are recreated by a resize.
    void UpdateCompositeDescriptorSets()
    {
        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            UpdateCompositeDescriptorSet(i);
        }
    }

    std::vector<uint32_t> ReadSpirV(const std::string& fileName)
    {
        auto& env = GetContext()->GetPlatformEnvironment();
        auto shadersPath = env.GetDirectoryPath(DirBase::openrct2, DirId::shaders);
        auto path = Path::Combine(shadersPath, fileName);

        auto fs = FileStream(path, FileMode::open);
        uint64_t fileLength = fs.GetLength();
        if (fileLength == 0 || (fileLength % 4) != 0)
        {
            throw std::runtime_error("Invalid SPIR-V shader file: " + path);
        }

        std::vector<uint32_t> code(static_cast<size_t>(fileLength) / 4);
        fs.Read(code.data(), fileLength);
        return code;
    }

    VkShaderModule CreateShaderModule(const std::vector<uint32_t>& code)
    {
        VkShaderModuleCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        info.codeSize = code.size() * sizeof(uint32_t);
        info.pCode = code.data();

        VkShaderModule module = VK_NULL_HANDLE;
        CheckVk(vkCreateShaderModule(_device, &info, nullptr, &module), "vkCreateShaderModule");
        return module;
    }

    // Builds the graphics pipeline that renders a fullscreen triangle and looks up each pixel's
    // final colour from the palette in the fragment shader, reading from the drawing context's
    // R8_UINT offscreen colour target - the Vulkan equivalent of the OpenGL renderer's
    // ApplyPaletteShader.
    void CreateGraphicsPipeline()
    {
        auto vertCode = ReadSpirV("applypalette_vk.vert.spv");
        auto fragCode = ReadSpirV("applypalette_vk.frag.spv");
        VkShaderModule vertModule = CreateShaderModule(vertCode);
        VkShaderModule fragModule = CreateShaderModule(fragCode);

        VkPipelineShaderStageCreateInfo vertStage{};
        vertStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertStage.module = vertModule;
        vertStage.pName = "main";

        VkPipelineShaderStageCreateInfo fragStage{};
        fragStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragStage.module = fragModule;
        fragStage.pName = "main";

        VkPipelineShaderStageCreateInfo stages[] = { vertStage, fragStage };

        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
        rasterizer.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample{};
        multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.blendEnable = VK_FALSE;
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT
            | VK_COLOR_COMPONENT_A_BIT;

        VkPipelineColorBlendStateCreateInfo colorBlend{};
        colorBlend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlend.attachmentCount = 1;
        colorBlend.pAttachments = &blendAttachment;

        VkDynamicState dynamicStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = 2;
        dynamicState.pDynamicStates = dynamicStates;

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = stages;
        pipelineInfo.pVertexInputState = &vertexInput;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisample;
        pipelineInfo.pColorBlendState = &colorBlend;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = _pipelineLayout;
        pipelineInfo.renderPass = _renderPass;
        pipelineInfo.subpass = 0;

        CheckVk(
            vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &_pipeline),
            "vkCreateGraphicsPipelines");

        vkDestroyShaderModule(_device, vertModule, nullptr);
        vkDestroyShaderModule(_device, fragModule, nullptr);
    }

    // Allocates one command buffer per frame-in-flight, recorded fresh every Present() call
    // (unlike the old "record once" scheme) since the offscreen rect/line draw calls now vary in
    // count every frame.
    void CreateCommandBuffers()
    {
        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = _commandPool;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = kFramesInFlight;
        CheckVk(vkAllocateCommandBuffers(_device, &alloc, _commandBuffers.data()), "vkAllocateCommandBuffers");
    }

    // Records this frame's offscreen rect/line draws (via the drawing context) followed by the
    // final palette-lookup composite pass into the swapchain image, then submits and presents.
    void Present()
    {
        if (_device == VK_NULL_HANDLE || _swapchain == VK_NULL_HANDLE || _width == 0 || _height == 0)
            return;

        CheckVk(vkWaitForFences(_device, 1, &_inFlightFences[_currentFrame], VK_TRUE, UINT64_MAX), "vkWaitForFences");

        uint32_t imageIndex = 0;
        VkResult acquire = vkAcquireNextImageKHR(
            _device, _swapchain, UINT64_MAX, _imageAvailable[_currentFrame], VK_NULL_HANDLE, &imageIndex);

        if (acquire == VK_ERROR_OUT_OF_DATE_KHR)
        {
            RecreateSwapchainAndResources();
            return;
        }
        CheckVk(acquire, "vkAcquireNextImageKHR");

        CheckVk(vkResetFences(_device, 1, &_inFlightFences[_currentFrame]), "vkResetFences");

        VkCommandBuffer cmd = _commandBuffers[_currentFrame];
        CheckVk(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer");

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        CheckVk(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer");

        _drawingContext.FlushCommandBuffers(cmd, _currentFrame);

        // The offscreen colour image is one of a ping-ponged pair (see HandleTransparency's
        // depth-peeling compositing) - GetColourImageView() may now point at a different image
        // than it did last frame, so this frame-in-flight's composite descriptor set must be
        // refreshed every frame, not just after a resize (its previous use, if any, is guaranteed
        // finished by the vkWaitForFences call above, so it's safe to update here).
        UpdateCompositeDescriptorSet(_currentFrame);

        VkRenderPassBeginInfo rpBegin{};
        rpBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        rpBegin.renderPass = _renderPass;
        rpBegin.framebuffer = _framebuffers[imageIndex];
        rpBegin.renderArea.extent = _swapchainExtent;
        vkCmdBeginRenderPass(cmd, &rpBegin, VK_SUBPASS_CONTENTS_INLINE);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
        vkCmdBindDescriptorSets(
            cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &_descriptorSets[_currentFrame], 0, nullptr);

        VkViewport viewport{};
        viewport.width = static_cast<float>(_swapchainExtent.width);
        viewport.height = static_cast<float>(_swapchainExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(cmd, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.extent = _swapchainExtent;
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        // Fullscreen triangle: 3 vertices, no vertex/index buffers (see applypalette_vk.vert).
        vkCmdDraw(cmd, 3, 1, 0, 0);

        vkCmdEndRenderPass(cmd);

        CheckVk(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

        VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.waitSemaphoreCount = 1;
        submit.pWaitSemaphores = &_imageAvailable[_currentFrame];
        submit.pWaitDstStageMask = &waitStage;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        submit.signalSemaphoreCount = 1;
        submit.pSignalSemaphores = &_renderFinished[_currentFrame];
        CheckVk(vkQueueSubmit(_graphicsQueue, 1, &submit, _inFlightFences[_currentFrame]), "vkQueueSubmit");

        VkPresentInfoKHR present{};
        present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        present.waitSemaphoreCount = 1;
        present.pWaitSemaphores = &_renderFinished[_currentFrame];
        present.swapchainCount = 1;
        present.pSwapchains = &_swapchain;
        present.pImageIndices = &imageIndex;

        VkResult presentResult = vkQueuePresentKHR(_graphicsQueue, &present);
        if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR)
        {
            RecreateSwapchainAndResources();
        }
        else
        {
            CheckVk(presentResult, "vkQueuePresentKHR");
        }

        _currentFrame = (_currentFrame + 1) % kFramesInFlight;
    }
};

std::unique_ptr<IDrawingEngine> Ui::CreateVulkanDrawingEngine(IUiContext& uiContext)
{
    return std::make_unique<VulkanDrawingEngine>(uiContext);
}

#endif
