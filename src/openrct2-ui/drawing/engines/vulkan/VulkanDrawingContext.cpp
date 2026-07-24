/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#ifndef DISABLE_VULKAN

#include "VulkanDrawingContext.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <openrct2/Context.h>
#include <openrct2/PlatformEnvironment.h>
#include <openrct2/core/EnumUtils.hpp>
#include <openrct2/core/FileStream.h>
#include <openrct2/core/Guard.hpp>
#include <openrct2/core/Path.hpp>
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/drawing/Drawing.String.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/TTF.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

namespace
{
    constexpr uint8_t kCSInside = 0b0000;
    constexpr uint8_t kCSLeft = 0b0001;
    constexpr uint8_t kCSRight = 0b0010;
    constexpr uint8_t kCSTop = 0b0100;
    constexpr uint8_t kCSBottom = 0b1000;

    // Same trick used to derive a NDC-z ordering from a monotonically increasing draw counter as
    // rect_vk.vert/line_vk.vert - kept here purely for documentation; the actual arithmetic lives
    // in the shaders themselves.

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

    VkShaderModule CreateShaderModule(VkDevice device, const std::vector<uint32_t>& code)
    {
        VkShaderModuleCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        info.codeSize = code.size() * sizeof(uint32_t);
        info.pCode = code.data();

        VkShaderModule module = VK_NULL_HANDLE;
        CheckVk(vkCreateShaderModule(device, &info, nullptr, &module), "vkCreateShaderModule");
        return module;
    }

    VkFormat FindSupportedDepthFormat(VkPhysicalDevice physicalDevice)
    {
        constexpr VkFormat candidates[] = { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT };
        for (auto format : candidates)
        {
            VkFormatProperties props{};
            vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);
            if ((props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0)
            {
                return format;
            }
        }
        throw std::runtime_error("No supported depth format found for Vulkan renderer.");
    }

    static auto EuclideanRemainder(const auto a, const auto b)
    {
        const auto r = a % b;
        return r >= 0 ? r : r + b;
    }
} // namespace

void VulkanDrawingContext::Initialise(
    VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool, RenderTarget* mainRT)
{
    _device = device;
    _physicalDevice = physicalDevice;
    _queue = queue;
    _commandPool = commandPool;
    _mainRT = mainRT;

    _depthFormat = FindSupportedDepthFormat(physicalDevice);

    _textureCache.Initialise(device, physicalDevice, queue, commandPool);

    // Descriptor set layout used by the rect pipeline: atlas array (binding 0) + palette lookup
    // texture (binding 1), both sampled as unsigned-integer textures (no filtering, matching
    // OpenGL's GL_NEAREST-only R8UI textures).
    VkDescriptorSetLayoutBinding atlasBinding{};
    atlasBinding.binding = 0;
    atlasBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    atlasBinding.descriptorCount = 1;
    atlasBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding paletteBinding{};
    paletteBinding.binding = 1;
    paletteBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    paletteBinding.descriptorCount = 1;
    paletteBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding bindings[] = { atlasBinding, paletteBinding };
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = bindings;
    CheckVk(
        vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_rectDescriptorSetLayout),
        "vkCreateDescriptorSetLayout(rect)");

    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = kVulkanFramesInFlight * 2;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = kVulkanFramesInFlight;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    CheckVk(vkCreateDescriptorPool(_device, &poolInfo, nullptr, &_descriptorPool), "vkCreateDescriptorPool(rect)");

    std::array<VkDescriptorSetLayout, kVulkanFramesInFlight> layouts{};
    layouts.fill(_rectDescriptorSetLayout);
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = _descriptorPool;
    allocInfo.descriptorSetCount = kVulkanFramesInFlight;
    allocInfo.pSetLayouts = layouts.data();
    CheckVk(vkAllocateDescriptorSets(_device, &allocInfo, _rectDescriptorSets.data()), "vkAllocateDescriptorSets(rect)");

    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_NEAREST;
    samplerInfo.minFilter = VK_FILTER_NEAREST;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    CheckVk(vkCreateSampler(_device, &samplerInfo, nullptr, &_atlasSampler), "vkCreateSampler(atlas)");
    CheckVk(vkCreateSampler(_device, &samplerInfo, nullptr, &_paletteSampler), "vkCreateSampler(palette)");

    CreatePipelines();
}

void VulkanDrawingContext::Destroy()
{
    if (_device == VK_NULL_HANDLE)
        return;

    DestroyOffscreenTargets();

    for (auto& buf : _rectInstanceBuffers)
        DestroyBuffer(_device, buf);
    for (auto& buf : _lineInstanceBuffers)
        DestroyBuffer(_device, buf);
    for (auto& buf : _stagingBuffers)
        DestroyBuffer(_device, buf);

    if (_atlasSampler != VK_NULL_HANDLE)
        vkDestroySampler(_device, _atlasSampler, nullptr);
    if (_paletteSampler != VK_NULL_HANDLE)
        vkDestroySampler(_device, _paletteSampler, nullptr);
    if (_descriptorPool != VK_NULL_HANDLE)
        vkDestroyDescriptorPool(_device, _descriptorPool, nullptr);
    if (_rectPipeline != VK_NULL_HANDLE)
        vkDestroyPipeline(_device, _rectPipeline, nullptr);
    if (_rectPipelineLayout != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(_device, _rectPipelineLayout, nullptr);
    if (_linePipeline != VK_NULL_HANDLE)
        vkDestroyPipeline(_device, _linePipeline, nullptr);
    if (_linePipelineLayout != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(_device, _linePipelineLayout, nullptr);
    if (_rectDescriptorSetLayout != VK_NULL_HANDLE)
        vkDestroyDescriptorSetLayout(_device, _rectDescriptorSetLayout, nullptr);
    if (_offscreenRenderPass != VK_NULL_HANDLE)
        vkDestroyRenderPass(_device, _offscreenRenderPass, nullptr);

    _textureCache.Destroy();

    _device = VK_NULL_HANDLE;
}

void VulkanDrawingContext::CreatePipelines()
{
    // Render pass: colour attachment persists between frames (LOAD_OP_LOAD) to support the
    // dirty-rectangle redraw optimisation inherited from the legacy CPU renderer model (only
    // invalidated regions are redrawn each frame; everything else keeps last frame's pixels) -
    // depth is cleared every frame since it only exists to encode this frame's paint order.
    VkAttachmentDescription colourAttachment{};
    colourAttachment.format = VK_FORMAT_R8_UINT;
    colourAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colourAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    colourAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colourAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colourAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colourAttachment.initialLayout = VK_IMAGE_LAYOUT_GENERAL;
    colourAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = _depthFormat;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_GENERAL;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkAttachmentReference colourRef{ 0, VK_IMAGE_LAYOUT_GENERAL };
    VkAttachmentReference depthRef{ 1, VK_IMAGE_LAYOUT_GENERAL };

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colourRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
        | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.srcAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_READ_BIT
        | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkAttachmentDescription attachments[] = { colourAttachment, depthAttachment };
    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = 2;
    renderPassInfo.pAttachments = attachments;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;
    CheckVk(vkCreateRenderPass(_device, &renderPassInfo, nullptr, &_offscreenRenderPass), "vkCreateRenderPass(offscreen)");

    // Rect pipeline layout: push constant (screen size) + atlas/palette descriptor set.
    VkPushConstantRange pushConstant{};
    pushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstant.offset = 0;
    pushConstant.size = sizeof(float) * 2;

    VkPipelineLayoutCreateInfo rectLayoutInfo{};
    rectLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    rectLayoutInfo.setLayoutCount = 1;
    rectLayoutInfo.pSetLayouts = &_rectDescriptorSetLayout;
    rectLayoutInfo.pushConstantRangeCount = 1;
    rectLayoutInfo.pPushConstantRanges = &pushConstant;
    CheckVk(vkCreatePipelineLayout(_device, &rectLayoutInfo, nullptr, &_rectPipelineLayout), "vkCreatePipelineLayout(rect)");

    VkPipelineLayoutCreateInfo lineLayoutInfo{};
    lineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    lineLayoutInfo.pushConstantRangeCount = 1;
    lineLayoutInfo.pPushConstantRanges = &pushConstant;
    CheckVk(vkCreatePipelineLayout(_device, &lineLayoutInfo, nullptr, &_linePipelineLayout), "vkCreatePipelineLayout(line)");

    // --- Rect pipeline ---
    {
        auto vertCode = ReadSpirV("rect_vk.vert.spv");
        auto fragCode = ReadSpirV("rect_vk.frag.spv");
        VkShaderModule vertModule = CreateShaderModule(_device, vertCode);
        VkShaderModule fragModule = CreateShaderModule(_device, fragCode);

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

        VkVertexInputBindingDescription binding{};
        binding.binding = 0;
        binding.stride = sizeof(VulkanDrawRectCommand);
        binding.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;

        std::array<VkVertexInputAttributeDescription, 11> attrs{};
        attrs[0] = { 0, 0, VK_FORMAT_R32G32B32A32_SINT, offsetof(VulkanDrawRectCommand, clip) };
        attrs[1] = { 1, 0, VK_FORMAT_R32_SINT, offsetof(VulkanDrawRectCommand, texColourAtlas) };
        attrs[2] = { 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(VulkanDrawRectCommand, texColourBounds) };
        attrs[3] = { 3, 0, VK_FORMAT_R32_SINT, offsetof(VulkanDrawRectCommand, texMaskAtlas) };
        attrs[4] = { 4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(VulkanDrawRectCommand, texMaskBounds) };
        attrs[5] = { 5, 0, VK_FORMAT_R32G32B32_SINT, offsetof(VulkanDrawRectCommand, palettes) };
        attrs[6] = { 6, 0, VK_FORMAT_R32_SINT, offsetof(VulkanDrawRectCommand, flags) };
        attrs[7] = { 7, 0, VK_FORMAT_R32_UINT, offsetof(VulkanDrawRectCommand, colour) };
        attrs[8] = { 8, 0, VK_FORMAT_R32G32B32A32_SINT, offsetof(VulkanDrawRectCommand, bounds) };
        attrs[9] = { 9, 0, VK_FORMAT_R32_SINT, offsetof(VulkanDrawRectCommand, depth) };
        attrs[10] = { 10, 0, VK_FORMAT_R32_SFLOAT, offsetof(VulkanDrawRectCommand, zoom) };

        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInput.vertexBindingDescriptionCount = 1;
        vertexInput.pVertexBindingDescriptions = &binding;
        vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
        vertexInput.pVertexAttributeDescriptions = attrs.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;

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

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;

        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.blendEnable = VK_FALSE;
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;

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
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlend;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = _rectPipelineLayout;
        pipelineInfo.renderPass = _offscreenRenderPass;
        pipelineInfo.subpass = 0;

        CheckVk(
            vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &_rectPipeline),
            "vkCreateGraphicsPipelines(rect)");

        vkDestroyShaderModule(_device, vertModule, nullptr);
        vkDestroyShaderModule(_device, fragModule, nullptr);
    }

    // --- Line pipeline ---
    {
        auto vertCode = ReadSpirV("line_vk.vert.spv");
        auto fragCode = ReadSpirV("line_vk.frag.spv");
        VkShaderModule vertModule = CreateShaderModule(_device, vertCode);
        VkShaderModule fragModule = CreateShaderModule(_device, fragCode);

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

        VkVertexInputBindingDescription binding{};
        binding.binding = 0;
        binding.stride = sizeof(VulkanDrawLineCommand);
        binding.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;

        std::array<VkVertexInputAttributeDescription, 3> attrs{};
        attrs[0] = { 0, 0, VK_FORMAT_R32G32B32A32_SINT, offsetof(VulkanDrawLineCommand, bounds) };
        attrs[1] = { 1, 0, VK_FORMAT_R32_UINT, offsetof(VulkanDrawLineCommand, colour) };
        attrs[2] = { 2, 0, VK_FORMAT_R32_SINT, offsetof(VulkanDrawLineCommand, depth) };

        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInput.vertexBindingDescriptionCount = 1;
        vertexInput.pVertexBindingDescriptions = &binding;
        vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
        vertexInput.pVertexAttributeDescriptions = attrs.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

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

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;

        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.blendEnable = VK_FALSE;
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;

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
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlend;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = _linePipelineLayout;
        pipelineInfo.renderPass = _offscreenRenderPass;
        pipelineInfo.subpass = 0;

        CheckVk(
            vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &_linePipeline),
            "vkCreateGraphicsPipelines(line)");

        vkDestroyShaderModule(_device, vertModule, nullptr);
        vkDestroyShaderModule(_device, fragModule, nullptr);
    }
}

void VulkanDrawingContext::DestroyOffscreenTargets()
{
    if (_device == VK_NULL_HANDLE)
        return;

    if (_offscreenFramebuffer != VK_NULL_HANDLE)
    {
        vkDestroyFramebuffer(_device, _offscreenFramebuffer, nullptr);
        _offscreenFramebuffer = VK_NULL_HANDLE;
    }
    if (_colourImageView != VK_NULL_HANDLE)
    {
        vkDestroyImageView(_device, _colourImageView, nullptr);
        _colourImageView = VK_NULL_HANDLE;
    }
    if (_colourImage != VK_NULL_HANDLE)
    {
        vkDestroyImage(_device, _colourImage, nullptr);
        _colourImage = VK_NULL_HANDLE;
    }
    if (_colourImageMemory != VK_NULL_HANDLE)
    {
        vkFreeMemory(_device, _colourImageMemory, nullptr);
        _colourImageMemory = VK_NULL_HANDLE;
    }
    if (_depthImageView != VK_NULL_HANDLE)
    {
        vkDestroyImageView(_device, _depthImageView, nullptr);
        _depthImageView = VK_NULL_HANDLE;
    }
    if (_depthImage != VK_NULL_HANDLE)
    {
        vkDestroyImage(_device, _depthImage, nullptr);
        _depthImage = VK_NULL_HANDLE;
    }
    if (_depthImageMemory != VK_NULL_HANDLE)
    {
        vkFreeMemory(_device, _depthImageMemory, nullptr);
        _depthImageMemory = VK_NULL_HANDLE;
    }
}

void VulkanDrawingContext::CreateOffscreenTargets()
{
    VkImageCreateInfo colourInfo{};
    colourInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    colourInfo.imageType = VK_IMAGE_TYPE_2D;
    colourInfo.format = VK_FORMAT_R8_UINT;
    colourInfo.extent = { _width, _height, 1 };
    colourInfo.mipLevels = 1;
    colourInfo.arrayLayers = 1;
    colourInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    colourInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    colourInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    colourInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    colourInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    CheckVk(vkCreateImage(_device, &colourInfo, nullptr, &_colourImage), "vkCreateImage(offscreen colour)");

    VkMemoryRequirements colourMemReq{};
    vkGetImageMemoryRequirements(_device, _colourImage, &colourMemReq);
    VkMemoryAllocateInfo colourAlloc{};
    colourAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    colourAlloc.allocationSize = colourMemReq.size;
    colourAlloc.memoryTypeIndex = FindMemoryType(
        _physicalDevice, colourMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    CheckVk(vkAllocateMemory(_device, &colourAlloc, nullptr, &_colourImageMemory), "vkAllocateMemory(offscreen colour)");
    CheckVk(vkBindImageMemory(_device, _colourImage, _colourImageMemory, 0), "vkBindImageMemory(offscreen colour)");

    VkImageViewCreateInfo colourViewInfo{};
    colourViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    colourViewInfo.image = _colourImage;
    colourViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    colourViewInfo.format = VK_FORMAT_R8_UINT;
    colourViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colourViewInfo.subresourceRange.levelCount = 1;
    colourViewInfo.subresourceRange.layerCount = 1;
    CheckVk(vkCreateImageView(_device, &colourViewInfo, nullptr, &_colourImageView), "vkCreateImageView(offscreen colour)");

    VkImageCreateInfo depthInfo{};
    depthInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    depthInfo.imageType = VK_IMAGE_TYPE_2D;
    depthInfo.format = _depthFormat;
    depthInfo.extent = { _width, _height, 1 };
    depthInfo.mipLevels = 1;
    depthInfo.arrayLayers = 1;
    depthInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    depthInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    depthInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    depthInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    depthInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    CheckVk(vkCreateImage(_device, &depthInfo, nullptr, &_depthImage), "vkCreateImage(offscreen depth)");

    VkMemoryRequirements depthMemReq{};
    vkGetImageMemoryRequirements(_device, _depthImage, &depthMemReq);
    VkMemoryAllocateInfo depthAlloc{};
    depthAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    depthAlloc.allocationSize = depthMemReq.size;
    depthAlloc.memoryTypeIndex = FindMemoryType(_physicalDevice, depthMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    CheckVk(vkAllocateMemory(_device, &depthAlloc, nullptr, &_depthImageMemory), "vkAllocateMemory(offscreen depth)");
    CheckVk(vkBindImageMemory(_device, _depthImage, _depthImageMemory, 0), "vkBindImageMemory(offscreen depth)");

    VkImageViewCreateInfo depthViewInfo{};
    depthViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    depthViewInfo.image = _depthImage;
    depthViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    depthViewInfo.format = _depthFormat;
    depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    depthViewInfo.subresourceRange.levelCount = 1;
    depthViewInfo.subresourceRange.layerCount = 1;
    CheckVk(vkCreateImageView(_device, &depthViewInfo, nullptr, &_depthImageView), "vkCreateImageView(offscreen depth)");

    VkImageView attachments[] = { _colourImageView, _depthImageView };
    VkFramebufferCreateInfo fbInfo{};
    fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbInfo.renderPass = _offscreenRenderPass;
    fbInfo.attachmentCount = 2;
    fbInfo.pAttachments = attachments;
    fbInfo.width = _width;
    fbInfo.height = _height;
    fbInfo.layers = 1;
    CheckVk(vkCreateFramebuffer(_device, &fbInfo, nullptr, &_offscreenFramebuffer), "vkCreateFramebuffer(offscreen)");

    // One-time transition to GENERAL (used permanently, for both attachment writes and later
    // shader sampling - see the class comment in VulkanDrawingContext.h) plus an initial clear
    // to palette index 0 (transparent), mirroring OpenGLDrawingEngine::Resize()'s explicit
    // Clear() call - subsequent frames rely on LOAD_OP_LOAD and only redraw dirty regions.
    VkCommandBufferAllocateInfo cmdAlloc{};
    cmdAlloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdAlloc.commandPool = _commandPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    CheckVk(vkAllocateCommandBuffers(_device, &cmdAlloc, &cmd), "vkAllocateCommandBuffers(offscreen init)");

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    CheckVk(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer(offscreen init)");

    TransitionImageLayout(cmd, _colourImage, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    TransitionImageLayout(cmd, _depthImage, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);

    VkClearColorValue clearColour{};
    clearColour.uint32[0] = 0;
    VkImageSubresourceRange colourRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
    vkCmdClearColorImage(cmd, _colourImage, VK_IMAGE_LAYOUT_GENERAL, &clearColour, 1, &colourRange);

    CheckVk(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(offscreen init)");

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    CheckVk(vkQueueSubmit(_queue, 1, &submit, VK_NULL_HANDLE), "vkQueueSubmit(offscreen init)");
    CheckVk(vkQueueWaitIdle(_queue), "vkQueueWaitIdle(offscreen init)");
    vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);
}

void VulkanDrawingContext::Resize(uint32_t width, uint32_t height)
{
    _width = width;
    _height = height;

    DestroyOffscreenTargets();
    if (width > 0 && height > 0)
    {
        CreateOffscreenTargets();
    }

    _rects.clear();
    _lines.clear();
}

void VulkanDrawingContext::StartNewDraw()
{
    Guard::Assert(_inDraw == false);
    _drawCount = 0;
    _inDraw = true;
}

void VulkanDrawingContext::FinishDraw()
{
    Guard::Assert(_inDraw == true);
    _inDraw = false;
}

void VulkanDrawingContext::EnsureInstanceBufferCapacity(VulkanBuffer& buffer, VkDeviceSize requiredSize)
{
    if (buffer.buffer != VK_NULL_HANDLE && buffer.size >= requiredSize)
        return;

    DestroyBuffer(_device, buffer);
    VkDeviceSize newSize = std::max<VkDeviceSize>(requiredSize, 1 << 14);
    buffer = CreateBuffer(
        _device, _physicalDevice, newSize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
}

void VulkanDrawingContext::FlushCommandBuffers(VkCommandBuffer cmd, uint32_t frameIndex)
{
    Guard::Assert(_inDraw == true);

    if (_offscreenFramebuffer == VK_NULL_HANDLE)
    {
        _rects.clear();
        _lines.clear();
        return;
    }

    // Upload any newly-seen sprite/glyph/text texture data before the render pass begins, then
    // make those writes visible to the fragment shader that will sample the atlas texture
    // during this same render pass (the atlas image stays permanently in GENERAL layout, so a
    // plain execution/memory barrier is sufficient here - no layout transition needed).
    _textureCache.FlushPendingUploads(cmd, _stagingBuffers[frameIndex]);

    VkMemoryBarrier uploadBarrier{};
    uploadBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    uploadBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    uploadBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(
        cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 1, &uploadBarrier, 0, nullptr, 0,
        nullptr);

    VkClearValue depthClear{};
    depthClear.depthStencil = { 1.0f, 0 };
    VkClearValue clearValues[] = { {}, depthClear };

    VkRenderPassBeginInfo rpBegin{};
    rpBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBegin.renderPass = _offscreenRenderPass;
    rpBegin.framebuffer = _offscreenFramebuffer;
    rpBegin.renderArea.extent = { _width, _height };
    rpBegin.clearValueCount = 2;
    rpBegin.pClearValues = clearValues;
    vkCmdBeginRenderPass(cmd, &rpBegin, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{};
    viewport.width = static_cast<float>(_width);
    viewport.height = static_cast<float>(_height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.extent = { _width, _height };
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    FlushLines(cmd, frameIndex);
    FlushRectangles(cmd, frameIndex);

    vkCmdEndRenderPass(cmd);
}

void VulkanDrawingContext::FlushLines(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (_lines.size() == 0)
        return;

    VkDeviceSize requiredSize = _lines.size() * sizeof(VulkanDrawLineCommand);
    EnsureInstanceBufferCapacity(_lineInstanceBuffers[frameIndex], requiredSize);
    std::memcpy(_lineInstanceBuffers[frameIndex].mapped, _lines.data(), requiredSize);

    float screenSize[2] = { static_cast<float>(_width), static_cast<float>(_height) };

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _linePipeline);
    vkCmdPushConstants(cmd, _linePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(screenSize), screenSize);

    VkBuffer vbo = _lineInstanceBuffers[frameIndex].buffer;
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &offset);
    vkCmdDraw(cmd, 2, static_cast<uint32_t>(_lines.size()), 0, 0);

    _lines.clear();
}

void VulkanDrawingContext::FlushRectangles(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (_rects.size() == 0)
        return;

    VkDeviceSize requiredSize = _rects.size() * sizeof(VulkanDrawRectCommand);
    EnsureInstanceBufferCapacity(_rectInstanceBuffers[frameIndex], requiredSize);
    std::memcpy(_rectInstanceBuffers[frameIndex].mapped, _rects.data(), requiredSize);

    VkDescriptorImageInfo atlasInfo{};
    atlasInfo.sampler = _atlasSampler;
    atlasInfo.imageView = _textureCache.GetAtlasImageView();
    atlasInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorImageInfo paletteInfo{};
    paletteInfo.sampler = _paletteSampler;
    paletteInfo.imageView = _textureCache.GetPaletteImageView();
    paletteInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet writes[2]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = _rectDescriptorSets[frameIndex];
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &atlasInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = _rectDescriptorSets[frameIndex];
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].pImageInfo = &paletteInfo;

    vkUpdateDescriptorSets(_device, 2, writes, 0, nullptr);

    float screenSize[2] = { static_cast<float>(_width), static_cast<float>(_height) };

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipeline);
    vkCmdPushConstants(cmd, _rectPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(screenSize), screenSize);
    vkCmdBindDescriptorSets(
        cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipelineLayout, 0, 1, &_rectDescriptorSets[frameIndex], 0, nullptr);

    VkBuffer vbo = _rectInstanceBuffers[frameIndex].buffer;
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &offset);
    vkCmdDraw(cmd, 4, static_cast<uint32_t>(_rects.size()), 0, 0);

    _rects.clear();
}

ScreenRect VulkanDrawingContext::CalculateClipping(const RenderTarget& rt) const
{
    // Identical "dirty hack" pointer-arithmetic trick used by the OpenGL renderer (see
    // OpenGLDrawingContext::CalculateClipping) - the engine maintains a dummy CPU-side _bits
    // buffer purely so RenderTarget::Crop()'d sub-render-targets can be mapped back to an
    // absolute screen-space rectangle this way, even though no real pixel data is ever written
    // to it any more.
    const RenderTarget* mainRT = _mainRT;
    const int32_t bytesPerRow = mainRT->LineStride();
    const int32_t bitsOffset = static_cast<int32_t>(rt.bits - mainRT->bits);

    const int32_t left = bitsOffset % bytesPerRow;
    const int32_t top = bitsOffset / bytesPerRow;
    const int32_t right = left + rt.width;
    const int32_t bottom = top + rt.height;

    return { { left, top }, { right, bottom } };
}

uint8_t VulkanDrawingContext::ComputeOutCode(const ScreenCoordsXY p, const ScreenCoordsXY topLeft, const ScreenCoordsXY bottomRight)
{
    uint8_t code = kCSInside;
    if (p.x < topLeft.x)
        code |= kCSLeft;
    else if (p.x > bottomRight.x)
        code |= kCSRight;
    if (p.y < topLeft.y)
        code |= kCSTop;
    else if (p.y > bottomRight.y)
        code |= kCSBottom;
    return code;
}

bool VulkanDrawingContext::CohenSutherlandLineClip(ScreenLine& line, const RenderTarget& rt)
{
    ScreenCoordsXY topLeft = { rt.x, rt.y };
    ScreenCoordsXY bottomRight = { rt.x + rt.width - 1, rt.y + rt.height - 1 };
    uint8_t outcode1 = ComputeOutCode(line.Point1, topLeft, bottomRight);
    uint8_t outcode2 = ComputeOutCode(line.Point2, topLeft, bottomRight);

    while (true)
    {
        if (outcode1 == kCSInside && outcode2 == kCSInside)
            return true;
        if (outcode1 & outcode2)
            return false;

        uint8_t outcodeOut = outcode2 > outcode1 ? outcode2 : outcode1;
        ScreenCoordsXY clipped;

        if (outcodeOut & kCSBottom)
        {
            clipped.x = line.Point1.x
                + (line.Point2.x - line.Point1.x) * (bottomRight.y - line.Point1.y) / (line.Point2.y - line.Point1.y);
            clipped.y = bottomRight.y;
        }
        else if (outcodeOut & kCSTop)
        {
            clipped.x = line.Point1.x
                + (line.Point2.x - line.Point1.x) * (topLeft.y - line.Point1.y) / (line.Point2.y - line.Point1.y);
            clipped.y = topLeft.y;
        }
        else if (outcodeOut & kCSRight)
        {
            clipped.y = line.Point1.y
                + (line.Point2.y - line.Point1.y) * (bottomRight.x - line.Point1.x) / (line.Point2.x - line.Point1.x);
            clipped.x = bottomRight.x;
        }
        else if (outcodeOut & kCSLeft)
        {
            clipped.y = line.Point1.y
                + (line.Point2.y - line.Point1.y) * (topLeft.x - line.Point1.x) / (line.Point2.x - line.Point1.x);
            clipped.x = topLeft.x;
        }

        if (outcodeOut == outcode1)
        {
            line.Point1 = clipped;
            outcode1 = ComputeOutCode(line.Point1, topLeft, bottomRight);
        }
        else
        {
            line.Point2 = clipped;
            outcode2 = ComputeOutCode(line.Point2, topLeft, bottomRight);
        }
    }
}

void VulkanDrawingContext::Clear(RenderTarget& rt, PaletteIndex paletteIndex)
{
    Guard::Assert(_inDraw == true);
    FillRect(rt, paletteIndex, rt.x, rt.y, rt.x + rt.width, rt.y + rt.height);
}

void VulkanDrawingContext::FillRect(
    RenderTarget& rt, PaletteIndex paletteIndex, int32_t left, int32_t top, int32_t right, int32_t bottom, bool crossHatch)
{
    Guard::Assert(_inDraw == true);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = 0;
    command.texColourBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.texMaskAtlas = 0;
    command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.palettes = { 0, 0, 0 };
    command.colour = EnumValue(paletteIndex);
    command.bounds = { left, top, right + 1, bottom + 1 };
    command.flags = VulkanDrawRectCommand::FLAG_NO_TEXTURE;
    command.depth = _drawCount++;
    command.zoom = 1.0f;

    if (crossHatch)
    {
        command.flags |= VulkanDrawRectCommand::FLAG_CROSS_HATCH;
    }
}

void VulkanDrawingContext::FilterRect(
    RenderTarget& rt, FilterPaletteID palette, int32_t left, int32_t top, int32_t right, int32_t bottom)
{
    Guard::Assert(_inDraw == true);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    // Phase 1 simplification (agreed): no depth-peeling transparency pass yet, so this is
    // folded into the same opaque batch as FillRect - a visual-only regression versus OpenGL
    // for the "see-through" preference, tracked for a later phase.
    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = 0;
    command.texColourBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.texMaskAtlas = 0;
    command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.palettes = { 0, 0, 0 };
    command.colour = static_cast<uint32_t>(VulkanTextureCache::PaletteToY(palette));
    command.bounds = { left, top, right + 1, bottom + 1 };
    command.flags = VulkanDrawRectCommand::FLAG_NO_TEXTURE;
    command.depth = _drawCount++;
    command.zoom = 1.0f;
}

void VulkanDrawingContext::DrawLine(RenderTarget& rt, PaletteIndex colour, const ScreenLine& line)
{
    Guard::Assert(_inDraw == true);

    const ZoomLevel zoom = rt.zoom_level;
    ScreenLine trimmedLine = { { zoom.ApplyInversedTo(line.GetX1()), zoom.ApplyInversedTo(line.GetY1()) },
                               { zoom.ApplyInversedTo(line.GetX2()), zoom.ApplyInversedTo(line.GetY2()) } };
    if (!CohenSutherlandLineClip(trimmedLine, rt))
        return;

    const ScreenRect clip = CalculateClipping(rt);
    const int32_t x1 = trimmedLine.GetX1() - rt.x + clip.GetLeft();
    const int32_t y1 = trimmedLine.GetY1() - rt.y + clip.GetTop();
    const int32_t x2 = trimmedLine.GetX2() - rt.x + clip.GetLeft();
    const int32_t y2 = trimmedLine.GetY2() - rt.y + clip.GetTop();

    VulkanDrawLineCommand& command = _lines.allocate();
    command.bounds = { x1, y1, x2, y2 };
    command.colour = static_cast<uint32_t>(colour);
    command.depth = _drawCount++;
}

void VulkanDrawingContext::DrawSprite(RenderTarget& rt, const ImageId imageId, const int32_t x, const int32_t y)
{
    Guard::Assert(_inDraw == true);

    auto g1Element = GfxGetG1Element(imageId);
    if (g1Element == nullptr)
        return;

    if (rt.zoom_level > ZoomLevel{ 0 })
    {
        if (g1Element->flags.has(G1Flag::hasZoomSprite))
        {
            RenderTarget zoomedRT;
            zoomedRT.bits = rt.bits;
            zoomedRT.x = rt.x;
            zoomedRT.y = rt.y;
            zoomedRT.height = rt.height;
            zoomedRT.width = rt.width;
            zoomedRT.pitch = rt.pitch;
            zoomedRT.zoom_level = rt.zoom_level - 1;
            DrawSprite(zoomedRT, imageId.WithIndex(imageId.GetIndex() - g1Element->zoomedOffset), x >> 1, y >> 1);
            return;
        }
        if (g1Element->flags.has(G1Flag::noZoomDraw))
        {
            return;
        }
    }

    auto texture = _textureCache.GetOrLoadImageTexture(imageId);

    int32_t left = x + g1Element->xOffset;
    int32_t top = y + g1Element->yOffset;

    int32_t xModifier = 0;
    int32_t yModifier = 0;
    int32_t widthModifier = 0;

    if (rt.zoom_level > ZoomLevel{ 0 })
    {
        const int32_t interval = rt.zoom_level.ApplyTo(1);

        xModifier = EuclideanRemainder(left, interval);
        xModifier = xModifier ? interval - xModifier : 0;
        yModifier = EuclideanRemainder(top, interval);
        widthModifier = EuclideanRemainder(left + g1Element->width, interval);
        widthModifier = widthModifier ? interval - widthModifier : 0;

        texture.coords.x += xModifier;
        texture.coords.y += (interval - 1) - yModifier;
    }

    left = rt.zoom_level.ApplyInversedTo(left + xModifier);
    top = rt.zoom_level.ApplyInversedTo(top);
    int32_t right = left + rt.zoom_level.ApplyInversedTo(g1Element->width + widthModifier);
    int32_t bottom = top + rt.zoom_level.ApplyInversedTo(g1Element->height + yModifier);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    const float zoom = rt.zoom_level >= ZoomLevel{ 0 } ? static_cast<float>(rt.zoom_level.ApplyTo(1))
                                                        : 1.0f / static_cast<float>(rt.zoom_level.ApplyInversedTo(1));

    int paletteCount;
    IVec3 palettes{ 0, 0, 0 };
    bool special = false;
    if (imageId.HasSecondary())
    {
        palettes.x = VulkanTextureCache::PaletteToY(static_cast<FilterPaletteID>(imageId.GetPrimary()));
        palettes.y = VulkanTextureCache::PaletteToY(static_cast<FilterPaletteID>(imageId.GetSecondary()));
        if (!imageId.HasTertiary())
        {
            paletteCount = 2;
        }
        else
        {
            paletteCount = 3;
            palettes.z = VulkanTextureCache::PaletteToY(static_cast<FilterPaletteID>(imageId.GetTertiary()));
        }
    }
    else if (imageId.IsRemap() || imageId.IsBlended())
    {
        paletteCount = 1;
        FilterPaletteID palette = static_cast<FilterPaletteID>(imageId.GetRemap());
        palettes.x = VulkanTextureCache::PaletteToY(palette);
        if (palette == FilterPaletteID::paletteWater)
        {
            special = true;
        }
    }
    else
    {
        paletteCount = 0;
    }

    // Phase 1 simplification: blended/water sprites (OpenGL's "transparent" batch) are folded
    // into the same opaque rects batch - see FilterRect() comment above.
    if (special || imageId.IsBlended())
    {
        VulkanDrawRectCommand& command = _rects.allocate();
        command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
        command.texColourAtlas = texture.index;
        command.texColourBounds = texture.coords;
        command.texMaskAtlas = texture.index;
        command.texMaskBounds = texture.coords;
        command.palettes = palettes;
        command.colour = static_cast<uint32_t>(palettes.x - (special ? 1 : 0));
        command.bounds = { left, top, right, bottom };
        command.flags = special ? 0 : (VulkanDrawRectCommand::FLAG_NO_TEXTURE | VulkanDrawRectCommand::FLAG_MASK);
        command.depth = _drawCount++;
        command.zoom = zoom;
    }
    else
    {
        VulkanDrawRectCommand& command = _rects.allocate();
        command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
        command.texColourAtlas = texture.index;
        command.texColourBounds = texture.coords;
        command.texMaskAtlas = 0;
        command.texMaskBounds = { 0.0f, 0.0f, texture.coords.z, texture.coords.w };
        command.palettes = palettes;
        command.colour = 0;
        command.bounds = { left, top, right, bottom };
        command.flags = paletteCount;
        command.depth = _drawCount++;
        command.zoom = zoom;
    }
}

void VulkanDrawingContext::DrawSpriteRawMasked(
    RenderTarget& rt, int32_t x, int32_t y, const ImageId maskImage, const ImageId colourImage)
{
    Guard::Assert(_inDraw == true);

    auto g1ElementMask = GfxGetG1Element(maskImage);
    auto g1ElementColour = GfxGetG1Element(colourImage);
    if (g1ElementMask == nullptr || g1ElementColour == nullptr)
        return;

    const auto textureMask = _textureCache.GetOrLoadImageTexture(maskImage);
    const auto textureColour = _textureCache.GetOrLoadImageTexture(colourImage);

    int32_t drawOffsetX = g1ElementMask->xOffset;
    int32_t drawOffsetY = g1ElementMask->yOffset;
    int32_t drawWidth = std::min(g1ElementMask->width, g1ElementColour->width);
    int32_t drawHeight = std::min(g1ElementMask->height, g1ElementColour->height);

    int32_t left = x + drawOffsetX;
    int32_t top = y + drawOffsetY;
    int32_t right = left + drawWidth;
    int32_t bottom = top + drawHeight;

    if (left > right)
        std::swap(left, right);
    if (top > bottom)
        std::swap(top, bottom);

    left = rt.zoom_level.ApplyInversedTo(left);
    top = rt.zoom_level.ApplyInversedTo(top);
    right = rt.zoom_level.ApplyInversedTo(right);
    bottom = rt.zoom_level.ApplyInversedTo(bottom);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    const float zoom = rt.zoom_level >= ZoomLevel{ 0 } ? static_cast<float>(rt.zoom_level.ApplyTo(1))
                                                        : 1.0f / static_cast<float>(rt.zoom_level.ApplyInversedTo(1));

    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = textureColour.index;
    command.texColourBounds = textureColour.coords;
    command.texMaskAtlas = textureMask.index;
    command.texMaskBounds = textureMask.coords;
    command.palettes = { 0, 0, 0 };
    command.flags = VulkanDrawRectCommand::FLAG_MASK;
    command.colour = 0;
    command.bounds = { left, top, right, bottom };
    command.depth = _drawCount++;
    command.zoom = zoom;
}

void VulkanDrawingContext::DrawSpriteSolid(RenderTarget& rt, const ImageId image, int32_t x, int32_t y, PaletteIndex colour)
{
    Guard::Assert(_inDraw == true);

    auto g1Element = GfxGetG1Element(image);
    if (g1Element == nullptr)
        return;

    const auto texture = _textureCache.GetOrLoadImageTexture(image);

    int32_t drawOffsetX = g1Element->xOffset;
    int32_t drawOffsetY = g1Element->yOffset;
    int32_t drawWidth = static_cast<uint16_t>(g1Element->width);
    int32_t drawHeight = static_cast<uint16_t>(g1Element->height);

    int32_t left = x + drawOffsetX;
    int32_t top = y + drawOffsetY;
    int32_t right = left + drawWidth;
    int32_t bottom = top + drawHeight;

    if (left > right)
        std::swap(left, right);
    if (top > bottom)
        std::swap(top, bottom);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = 0;
    command.texColourBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.texMaskAtlas = texture.index;
    command.texMaskBounds = texture.coords;
    command.palettes = { 0, 0, 0 };
    command.flags = VulkanDrawRectCommand::FLAG_NO_TEXTURE | VulkanDrawRectCommand::FLAG_MASK;
    command.colour = static_cast<uint32_t>(colour);
    command.bounds = { left, top, right, bottom };
    command.depth = _drawCount++;
    command.zoom = 1.0f;
}

void VulkanDrawingContext::DrawGlyph(RenderTarget& rt, const ImageId image, int32_t x, int32_t y, const PaletteMap& palette)
{
    Guard::Assert(_inDraw == true);

    auto g1Element = GfxGetG1Element(image);
    if (g1Element == nullptr)
        return;

    const auto texture = _textureCache.GetOrLoadGlyphTexture(image, palette);

    int32_t left = x + g1Element->xOffset;
    int32_t top = y + g1Element->yOffset;
    int32_t right = left + static_cast<uint16_t>(g1Element->width);
    int32_t bottom = top + static_cast<uint16_t>(g1Element->height);

    if (left > right)
        std::swap(left, right);
    if (top > bottom)
        std::swap(top, bottom);

    left = rt.zoom_level.ApplyInversedTo(left);
    top = rt.zoom_level.ApplyInversedTo(top);
    right = rt.zoom_level.ApplyInversedTo(right);
    bottom = rt.zoom_level.ApplyInversedTo(bottom);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    const float zoom = rt.zoom_level >= ZoomLevel{ 0 } ? static_cast<float>(rt.zoom_level.ApplyTo(1))
                                                        : 1.0f / static_cast<float>(rt.zoom_level.ApplyInversedTo(1));

    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = texture.index;
    command.texColourBounds = texture.coords;
    command.texMaskAtlas = 0;
    command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.palettes = { 0, 0, 0 };
    command.flags = 0;
    command.colour = 0;
    command.bounds = { left, top, right, bottom };
    command.depth = _drawCount++;
    command.zoom = zoom;
}

void VulkanDrawingContext::DrawTTFBitmap(
    RenderTarget& rt, const TextDrawInfo& info, TTFSurface* surface, int32_t x, int32_t y, uint8_t hintingThreshold)
{
    Guard::Assert(_inDraw == true);

#ifndef DISABLE_TTF
    auto baseId = static_cast<uint32_t>(0x7FFFF) - 1024;
    auto imageId = baseId + _ttfGlId;
    _textureCache.InvalidateImage(imageId);
    const auto texture = _textureCache.GetOrLoadBitmapTexture(imageId, surface->pixels, surface->w, surface->h);
    _ttfGlId++;
    if (_ttfGlId >= 1023)
    {
        _ttfGlId = 0;
    }

    int32_t drawWidth = static_cast<uint16_t>(surface->w);
    int32_t drawHeight = static_cast<uint16_t>(surface->h);

    int32_t left = x;
    int32_t top = y;
    int32_t right = left + drawWidth;
    int32_t bottom = top + drawHeight;

    if (left > right)
        std::swap(left, right);
    if (top > bottom)
        std::swap(top, bottom);

    const ScreenRect clip = CalculateClipping(rt);
    left += clip.GetLeft() - rt.x;
    top += clip.GetTop() - rt.y;
    right += clip.GetLeft() - rt.x;
    bottom += clip.GetTop() - rt.y;

    if (info.colourFlags.has(ColourFlag::withOutline))
    {
        std::array<IVec4, 4> boundsArr = { {
            { left + 1, top, right + 1, bottom },
            { left - 1, top, right - 1, bottom },
            { left, top + 1, right, bottom + 1 },
            { left, top - 1, right, bottom - 1 },
        } };
        for (auto b : boundsArr)
        {
            VulkanDrawRectCommand& command = _rects.allocate();
            command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
            command.texColourAtlas = texture.index;
            command.texColourBounds = texture.coords;
            command.texMaskAtlas = 0;
            command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
            command.palettes = { 0, 0, 0 };
            command.flags = VulkanDrawRectCommand::FLAG_TTF_TEXT;
            command.colour = static_cast<uint32_t>(info.palette.shadowOutline);
            command.bounds = b;
            command.depth = _drawCount++;
            command.zoom = 1.0f;
        }
    }
    if (info.colourFlags.has(ColourFlag::inset))
    {
        VulkanDrawRectCommand& command = _rects.allocate();
        command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
        command.texColourAtlas = texture.index;
        command.texColourBounds = texture.coords;
        command.texMaskAtlas = 0;
        command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
        command.palettes = { 0, 0, 0 };
        command.flags = VulkanDrawRectCommand::FLAG_TTF_TEXT;
        command.colour = static_cast<uint32_t>(info.palette.shadowOutline);
        command.bounds = { left + 1, top + 1, right + 1, bottom + 1 };
        command.depth = _drawCount++;
        command.zoom = 1.0f;
    }

    // Phase 1 simplification: hinted text (OpenGL's "transparent" batch) folded into the
    // opaque batch - see FilterRect() comment above.
    VulkanDrawRectCommand& command = _rects.allocate();
    command.clip = { clip.GetLeft(), clip.GetTop(), clip.GetRight(), clip.GetBottom() };
    command.texColourAtlas = texture.index;
    command.texColourBounds = texture.coords;
    command.texMaskAtlas = 0;
    command.texMaskBounds = { 0.0f, 0.0f, 0.0f, 0.0f };
    command.palettes = { 0, 0, 0 };
    command.flags = VulkanDrawRectCommand::FLAG_TTF_TEXT | (static_cast<int32_t>(hintingThreshold) << 8);
    command.colour = static_cast<uint32_t>(info.palette.fill);
    command.bounds = { left, top, right, bottom };
    command.depth = _drawCount++;
    command.zoom = 1.0f;
#endif // DISABLE_TTF
}

#endif // DISABLE_VULKAN
