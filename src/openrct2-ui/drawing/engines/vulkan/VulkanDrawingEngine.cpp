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

#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>

#include <openrct2/Context.h>
#include <openrct2/Diagnostic.h>
#include <openrct2/PlatformEnvironment.h>
#include <openrct2/core/EnumUtils.hpp>
#include <openrct2/core/FileStream.h>
#include <openrct2/core/Path.hpp>
#include <openrct2/drawing/X8DrawingEngine.h>
#include <openrct2/ui/UiContext.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

namespace
{
    constexpr uint32_t kFramesInFlight = 2;

    uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties)
    {
        VkPhysicalDeviceMemoryProperties memProperties{};
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
        {
            if ((typeFilter & (1u << i)) != 0
                && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("Unable to find suitable Vulkan memory type.");
    }

    void CheckVk(VkResult result, const char* what)
    {
        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(std::string(what) + " failed with error code " + std::to_string(result));
        }
    }
} // namespace

class VulkanDrawingEngine final : public X8DrawingEngine
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
    std::vector<VkCommandBuffer> _commandBuffers;

    std::array<VkSemaphore, kFramesInFlight> _imageAvailable{};
    std::array<VkSemaphore, kFramesInFlight> _renderFinished{};
    std::array<VkFence, kFramesInFlight> _inFlightFences{};
    uint32_t _currentFrame = 0;

    std::vector<VkImageView> _swapchainImageViews;
    std::vector<VkFramebuffer> _framebuffers;

    // Device-lifetime shader/pipeline resources: created once and reused across swapchain
    // recreations (resize), since they don't depend on the swapchain extent.
    VkRenderPass _renderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout _descriptorSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkSampler _paletteIndexSampler = VK_NULL_HANDLE;
    VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, kFramesInFlight> _descriptorSets{};

    // Each frame-in-flight gets its own upload buffer so the CPU can write the next frame's
    // pixel data while the GPU is still consuming a previous frame's buffer, without a race.
    // The buffer now holds raw 8-bit palette indices (1 byte/pixel) rather than pre-converted
    // BGRA colour (4 bytes/pixel): the palette lookup itself is done on the GPU by the
    // fragment shader (data/shaders/applypalette_vk.frag), mirroring the OpenGL renderer.
    std::array<VkBuffer, kFramesInFlight> _uploadBuffers{};
    std::array<VkDeviceMemory, kFramesInFlight> _uploadBufferMemories{};
    std::array<void*, kFramesInFlight> _uploadBuffersMapped{};
    VkDeviceSize _uploadBufferSize = 0;

    // Palette-index sampled image (R8_UINT), one per frame-in-flight for the same reason as
    // the upload buffers above.
    std::array<VkImage, kFramesInFlight> _paletteIndexImages{};
    std::array<VkDeviceMemory, kFramesInFlight> _paletteIndexImageMemories{};
    std::array<VkImageView, kFramesInFlight> _paletteIndexImageViews{};

    // Palette colour lookup table, uploaded to a uniform buffer read by the fragment shader.
    // One per frame-in-flight so an update while a previous frame is still in flight can't race.
    std::array<VkBuffer, kFramesInFlight> _paletteBuffers{};
    std::array<VkDeviceMemory, kFramesInFlight> _paletteBufferMemories{};
    std::array<void*, kFramesInFlight> _paletteBuffersMapped{};
    std::array<float, 256 * 4> _paletteData{};


    bool _useVsync = true;

public:
    explicit VulkanDrawingEngine(IUiContext& uiContext)
        : X8DrawingEngine(uiContext)
        , _uiContext(uiContext)
    {
        _window = static_cast<SDL_Window*>(_uiContext.GetWindow());
    }

    ~VulkanDrawingEngine() override
    {
        DestroyVulkan();
    }

    void Initialise() override
    {
        X8DrawingEngine::Initialise();
        CreateVulkan();
        LOG_VERBOSE("Vulkan renderer initialised (%ux%u).", _width, _height);
    }

    void Resize(uint32_t width, uint32_t height) override
    {
        X8DrawingEngine::Resize(width, height);
        RecreateSwapchainAndResources();
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

    void EndDraw() override
    {
        X8DrawingEngine::EndDraw();
        Present();
    }

private:
    void CreateVulkan()
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

        if (_width > 0 && _height > 0)
        {
            RecreateSwapchainAndResources();
        }
    }

    void DestroyVulkan()
    {
        if (_device != VK_NULL_HANDLE)
        {
            vkDeviceWaitIdle(_device);
        }

        DestroySwapchainAndResources();

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

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            if (_uploadBuffers[i] != VK_NULL_HANDLE)
            {
                if (_uploadBuffersMapped[i] != nullptr)
                {
                    vkUnmapMemory(_device, _uploadBufferMemories[i]);
                    _uploadBuffersMapped[i] = nullptr;
                }
                vkDestroyBuffer(_device, _uploadBuffers[i], nullptr);
                _uploadBuffers[i] = VK_NULL_HANDLE;
            }
            if (_uploadBufferMemories[i] != VK_NULL_HANDLE)
            {
                vkFreeMemory(_device, _uploadBufferMemories[i], nullptr);
                _uploadBufferMemories[i] = VK_NULL_HANDLE;
            }
        }

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            if (_paletteIndexImageViews[i] != VK_NULL_HANDLE)
            {
                vkDestroyImageView(_device, _paletteIndexImageViews[i], nullptr);
                _paletteIndexImageViews[i] = VK_NULL_HANDLE;
            }
            if (_paletteIndexImages[i] != VK_NULL_HANDLE)
            {
                vkDestroyImage(_device, _paletteIndexImages[i], nullptr);
                _paletteIndexImages[i] = VK_NULL_HANDLE;
            }
            if (_paletteIndexImageMemories[i] != VK_NULL_HANDLE)
            {
                vkFreeMemory(_device, _paletteIndexImageMemories[i], nullptr);
                _paletteIndexImageMemories[i] = VK_NULL_HANDLE;
            }
        }

        if (!_commandBuffers.empty())
        {
            vkFreeCommandBuffers(_device, _commandPool, static_cast<uint32_t>(_commandBuffers.size()), _commandBuffers.data());
            _commandBuffers.clear();
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
        CreateUploadResources();
        CreateCommandBuffers();
    }

    // Runs a single one-shot command buffer synchronously. Only used during (re)creation of
    // swapchain/image resources, never in the per-frame hot path.
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
        CheckVk(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(_physicalDevice, _surface, &caps), "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");

        uint32_t formatCount = 0;
        CheckVk(vkGetPhysicalDeviceSurfaceFormatsKHR(_physicalDevice, _surface, &formatCount, nullptr), "vkGetPhysicalDeviceSurfaceFormatsKHR(count)");
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        CheckVk(vkGetPhysicalDeviceSurfaceFormatsKHR(_physicalDevice, _surface, &formatCount, formats.data()), "vkGetPhysicalDeviceSurfaceFormatsKHR(list)");

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
        CheckVk(vkGetPhysicalDeviceSurfacePresentModesKHR(_physicalDevice, _surface, &presentModeCount, nullptr), "vkGetPhysicalDeviceSurfacePresentModesKHR(count)");
        std::vector<VkPresentModeKHR> presentModes(presentModeCount);
        CheckVk(vkGetPhysicalDeviceSurfacePresentModesKHR(_physicalDevice, _surface, &presentModeCount, presentModes.data()), "vkGetPhysicalDeviceSurfacePresentModesKHR(list)");

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
        CheckVk(vkGetSwapchainImagesKHR(_device, _swapchain, &swapCount, _swapchainImages.data()), "vkGetSwapchainImagesKHR(list)");
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
    // around it (unlike the old copy/blit path).
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
            CheckVk(vkBindBufferMemory(_device, _paletteBuffers[i], _paletteBufferMemories[i], 0), "vkBindBufferMemory(palette)");
            CheckVk(
                vkMapMemory(_device, _paletteBufferMemories[i], 0, bufferSize, 0, &_paletteBuffersMapped[i]),
                "vkMapMemory(palette)");

            // Initialise with whatever palette data we currently have (may just be the
            // zero-initialised default if SetPalette() hasn't been called yet).
            std::memcpy(_paletteBuffersMapped[i], _paletteData.data(), bufferSize);
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
    // final colour from the palette in the fragment shader - the Vulkan equivalent of the
    // OpenGL renderer's ApplyPaletteShader, replacing the old CPU-side palette conversion.
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
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
            | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

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

    // Pre-records one command buffer per (swapchain image, frame-in-flight) combination.
    // Since the CPU-side work (palette conversion) and copy/blit regions never change between
    // frames, we record the transitions and copy/blit commands exactly once here instead of
    // re-recording (vkBegin/vkCmd.../vkEnd) every single frame, which is the classic Vulkan
    // "record once, submit many" pattern and removes significant per-frame CPU/API overhead.
    void CreateCommandBuffers()
    {
        const auto imageCount = static_cast<uint32_t>(_swapchainImages.size());
        _commandBuffers.resize(imageCount * kFramesInFlight);

        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = _commandPool;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = static_cast<uint32_t>(_commandBuffers.size());
        CheckVk(vkAllocateCommandBuffers(_device, &alloc, _commandBuffers.data()), "vkAllocateCommandBuffers");

        for (uint32_t imageIndex = 0; imageIndex < imageCount; imageIndex++)
        {
            for (uint32_t frameIndex = 0; frameIndex < kFramesInFlight; frameIndex++)
            {
                RecordCommandBuffer(_commandBuffers[imageIndex * kFramesInFlight + frameIndex], imageIndex, frameIndex);
            }
        }
    }

    // Creates resources that are sized to the current render resolution (_width x _height) and
    // therefore need to be recreated on resize: the per-frame-in-flight upload buffers and the
    // palette-index sampled image the fragment shader reads from.
    void CreateUploadResources()
    {
        _uploadBufferSize = static_cast<VkDeviceSize>(_width) * static_cast<VkDeviceSize>(_height);

        for (uint32_t i = 0; i < kFramesInFlight; i++)
        {
            VkBufferCreateInfo bufferInfo{};
            bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            bufferInfo.size = _uploadBufferSize;
            bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            CheckVk(vkCreateBuffer(_device, &bufferInfo, nullptr, &_uploadBuffers[i]), "vkCreateBuffer(upload)");

            VkMemoryRequirements bufferMemReq{};
            vkGetBufferMemoryRequirements(_device, _uploadBuffers[i], &bufferMemReq);

            VkMemoryAllocateInfo bufferAlloc{};
            bufferAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            bufferAlloc.allocationSize = bufferMemReq.size;
            bufferAlloc.memoryTypeIndex = FindMemoryType(
                _physicalDevice, bufferMemReq.memoryTypeBits,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            CheckVk(vkAllocateMemory(_device, &bufferAlloc, nullptr, &_uploadBufferMemories[i]), "vkAllocateMemory(upload)");
            CheckVk(vkBindBufferMemory(_device, _uploadBuffers[i], _uploadBufferMemories[i], 0), "vkBindBufferMemory(upload)");
            CheckVk(
                vkMapMemory(_device, _uploadBufferMemories[i], 0, _uploadBufferSize, 0, &_uploadBuffersMapped[i]),
                "vkMapMemory(upload)");

            VkImageCreateInfo imageInfo{};
            imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.format = VK_FORMAT_R8_UINT;
            imageInfo.extent = { _width, _height, 1 };
            imageInfo.mipLevels = 1;
            imageInfo.arrayLayers = 1;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            CheckVk(vkCreateImage(_device, &imageInfo, nullptr, &_paletteIndexImages[i]), "vkCreateImage(paletteIndex)");

            VkMemoryRequirements imageMemReq{};
            vkGetImageMemoryRequirements(_device, _paletteIndexImages[i], &imageMemReq);

            VkMemoryAllocateInfo imageAlloc{};
            imageAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            imageAlloc.allocationSize = imageMemReq.size;
            imageAlloc.memoryTypeIndex = FindMemoryType(
                _physicalDevice, imageMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            CheckVk(vkAllocateMemory(_device, &imageAlloc, nullptr, &_paletteIndexImageMemories[i]), "vkAllocateMemory(paletteIndex)");
            CheckVk(vkBindImageMemory(_device, _paletteIndexImages[i], _paletteIndexImageMemories[i], 0), "vkBindImageMemory(paletteIndex)");

            VkImageViewCreateInfo viewInfo{};
            viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            viewInfo.image = _paletteIndexImages[i];
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = VK_FORMAT_R8_UINT;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.layerCount = 1;
            CheckVk(vkCreateImageView(_device, &viewInfo, nullptr, &_paletteIndexImageViews[i]), "vkCreateImageView(paletteIndex)");

            // One-time transition so every pre-recorded command buffer can assume a fixed
            // "previous layout" of SHADER_READ_ONLY_OPTIMAL for this image, every frame.
            RunOneTimeCommands([this, i](VkCommandBuffer cmd) {
                TransitionImage(
                    cmd, _paletteIndexImages[i], VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            });

            // Bind this frame-in-flight's image/sampler/palette buffer into its descriptor set.
            VkDescriptorImageInfo imgInfo{};
            imgInfo.sampler = _paletteIndexSampler;
            imgInfo.imageView = _paletteIndexImageViews[i];
            imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = _paletteBuffers[i];
            bufInfo.offset = 0;
            bufInfo.range = VK_WHOLE_SIZE;

            VkWriteDescriptorSet writes[2]{};
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = _descriptorSets[i];
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[0].pImageInfo = &imgInfo;

            writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[1].dstSet = _descriptorSets[i];
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[1].pBufferInfo = &bufInfo;

            vkUpdateDescriptorSets(_device, 2, writes, 0, nullptr);
        }
    }

    void TransitionImage(VkCommandBuffer cmd, VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout)
    {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = 0;

        if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
        {
            srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)
        {
            srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR)
        {
            srcStage = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        {
            srcStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        }

        if (newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
        {
            dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        }
        else if (newLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)
        {
            dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        }
        else if (newLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR)
        {
            dstStage = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        }
        else if (newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        {
            dstStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        }

        vkCmdPipelineBarrier(
            cmd, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    void UploadFrameToBuffer()
    {
        void* mapped = _uploadBuffersMapped[_currentFrame];
        if (_uploadBuffers[_currentFrame] == VK_NULL_HANDLE || mapped == nullptr)
            return;

        // The fragment shader now performs the palette lookup on the GPU (see
        // data/shaders/applypalette_vk.frag), mirroring the OpenGL renderer's ApplyPaletteShader.
        // All the CPU needs to do each frame is copy the raw 8-bit palette-index framebuffer
        // across - a single memcpy, not a per-pixel lookup - so no worker-thread pool is needed
        // here any more.
        std::memcpy(mapped, _bits, static_cast<size_t>(_width) * _height);
    }

    // Records the fixed sequence of commands for a given (swapchain image, frame-in-flight)
    // pair: upload this frame-in-flight's palette-index data into its sampled image, then run
    // the palette-lookup render pass directly into the swapchain image. Called only during
    // setup/resize, never per-frame (the classic Vulkan "record once, submit many" pattern).
    // The palette-index image is assumed to be in SHADER_READ_ONLY_OPTIMAL before this runs
    // (guaranteed by the one-time init transition plus the fact this always ends by
    // transitioning back to it). The render pass handles all of the swapchain image's layout
    // transitions via its initialLayout/finalLayout, so no manual barriers are needed around it.
    void RecordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex, uint32_t frameIndex)
    {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        CheckVk(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer");

        TransitionImage(
            cmd, _paletteIndexImages[frameIndex], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

        VkBufferImageCopy copy{};
        copy.bufferOffset = 0;
        copy.bufferRowLength = 0;
        copy.bufferImageHeight = 0;
        copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copy.imageSubresource.mipLevel = 0;
        copy.imageSubresource.baseArrayLayer = 0;
        copy.imageSubresource.layerCount = 1;
        copy.imageOffset = { 0, 0, 0 };
        copy.imageExtent = { _width, _height, 1 };
        vkCmdCopyBufferToImage(
            cmd, _uploadBuffers[frameIndex], _paletteIndexImages[frameIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

        TransitionImage(
            cmd, _paletteIndexImages[frameIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        VkRenderPassBeginInfo rpBegin{};
        rpBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        rpBegin.renderPass = _renderPass;
        rpBegin.framebuffer = _framebuffers[imageIndex];
        rpBegin.renderArea.extent = _swapchainExtent;
        vkCmdBeginRenderPass(cmd, &rpBegin, VK_SUBPASS_CONTENTS_INLINE);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
        vkCmdBindDescriptorSets(
            cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &_descriptorSets[frameIndex], 0, nullptr);

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
    }

    // Submits a pre-recorded command buffer for the current frame-in-flight. This is the only
    // per-frame Vulkan "work" beyond the palette-index upload: no recording, no barrier setup
    // here.
    void SubmitCommandBuffer(VkCommandBuffer cmd)
    {
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
    }

    void Present()
    {
        if (_device == VK_NULL_HANDLE || _swapchain == VK_NULL_HANDLE || _width == 0 || _height == 0)
            return;

        CheckVk(vkWaitForFences(_device, 1, &_inFlightFences[_currentFrame], VK_TRUE, UINT64_MAX), "vkWaitForFences");
        CheckVk(vkResetFences(_device, 1, &_inFlightFences[_currentFrame]), "vkResetFences");

        uint32_t imageIndex = 0;
        VkResult acquire = vkAcquireNextImageKHR(
            _device,
            _swapchain,
            UINT64_MAX,
            _imageAvailable[_currentFrame],
            VK_NULL_HANDLE,
            &imageIndex);

        if (acquire == VK_ERROR_OUT_OF_DATE_KHR)
        {
            RecreateSwapchainAndResources();
            return;
        }
        CheckVk(acquire, "vkAcquireNextImageKHR");

        UploadFrameToBuffer();
        SubmitCommandBuffer(_commandBuffers[imageIndex * kFramesInFlight + _currentFrame]);

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
