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

#include "VulkanTransparencyDepth.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
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

    // Descriptor set layout used by the rect pipeline (shared by both the opaque and the
    // depth-peeling transparent pipeline variants): atlas array (binding 0) + palette lookup
    // texture (binding 1), both sampled as unsigned-integer textures (no filtering, matching
    // OpenGL's GL_NEAREST-only R8UI textures), plus a peeling-reference depth texture (binding 2,
    // only actually sampled when the "uPeeling" push constant is set - always bound to something
    // valid regardless, see FlushRectangles/HandleTransparency).
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

    VkDescriptorSetLayoutBinding peelingBinding{};
    peelingBinding.binding = 2;
    peelingBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    peelingBinding.descriptorCount = 1;
    peelingBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding bindings[] = { atlasBinding, paletteBinding, peelingBinding };
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = bindings;
    CheckVk(
        vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_rectDescriptorSetLayout),
        "vkCreateDescriptorSetLayout(rect)");

    // Descriptor set layout for the apply-transparency composite pass: opaque colour+depth,
    // transparent colour+depth, palette, blend-palette (6 combined image samplers).
    VkDescriptorSetLayoutBinding transparencyBindings[6]{};
    for (uint32_t i = 0; i < 6; i++)
    {
        transparencyBindings[i].binding = i;
        transparencyBindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        transparencyBindings[i].descriptorCount = 1;
        transparencyBindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo transparencyLayoutInfo{};
    transparencyLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    transparencyLayoutInfo.bindingCount = 6;
    transparencyLayoutInfo.pBindings = transparencyBindings;
    CheckVk(
        vkCreateDescriptorSetLayout(_device, &transparencyLayoutInfo, nullptr, &_transparencyDescriptorSetLayout),
        "vkCreateDescriptorSetLayout(transparency)");

    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = kVulkanFramesInFlight * (3 + 6);

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = kVulkanFramesInFlight * 2;
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

    std::array<VkDescriptorSetLayout, kVulkanFramesInFlight> transparencyLayouts{};
    transparencyLayouts.fill(_transparencyDescriptorSetLayout);
    VkDescriptorSetAllocateInfo transparencyAllocInfo{};
    transparencyAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    transparencyAllocInfo.descriptorPool = _descriptorPool;
    transparencyAllocInfo.descriptorSetCount = kVulkanFramesInFlight;
    transparencyAllocInfo.pSetLayouts = transparencyLayouts.data();
    CheckVk(
        vkAllocateDescriptorSets(_device, &transparencyAllocInfo, _transparencyDescriptorSets.data()),
        "vkAllocateDescriptorSets(transparency)");

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

    // UINT64_MAX never matches a real VulkanTextureCache::GetAtlasVersion() value (which starts
    // at 0 and only ever increments), so this forces the first FlushRectangles/HandleTransparency
    // call for each frame-in-flight to always (re)write its atlas binding at least once.
    _rectAtlasBoundVersion.fill(std::numeric_limits<uint64_t>::max());

    CreatePipelines();
}

void VulkanDrawingContext::Destroy()
{
    if (_device == VK_NULL_HANDLE)
        return;

    DestroyOffscreenTargets();

    for (auto& buf : _rectInstanceBuffers)
        DestroyBuffer(_device, buf);
    for (auto& buf : _transparentRectInstanceBuffers)
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
    if (_rectPipelineTransparent != VK_NULL_HANDLE)
        vkDestroyPipeline(_device, _rectPipelineTransparent, nullptr);
    if (_rectPipelineLayout != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(_device, _rectPipelineLayout, nullptr);
    if (_linePipeline != VK_NULL_HANDLE)
        vkDestroyPipeline(_device, _linePipeline, nullptr);
    if (_linePipelineLayout != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(_device, _linePipelineLayout, nullptr);
    if (_transparencyPipeline != VK_NULL_HANDLE)
        vkDestroyPipeline(_device, _transparencyPipeline, nullptr);
    if (_transparencyPipelineLayout != VK_NULL_HANDLE)
        vkDestroyPipelineLayout(_device, _transparencyPipelineLayout, nullptr);
    if (_rectDescriptorSetLayout != VK_NULL_HANDLE)
        vkDestroyDescriptorSetLayout(_device, _rectDescriptorSetLayout, nullptr);
    if (_transparencyDescriptorSetLayout != VK_NULL_HANDLE)
        vkDestroyDescriptorSetLayout(_device, _transparencyDescriptorSetLayout, nullptr);
    if (_offscreenRenderPass != VK_NULL_HANDLE)
        vkDestroyRenderPass(_device, _offscreenRenderPass, nullptr);
    if (_transparentRenderPass != VK_NULL_HANDLE)
        vkDestroyRenderPass(_device, _transparentRenderPass, nullptr);
    if (_mixRenderPass != VK_NULL_HANDLE)
        vkDestroyRenderPass(_device, _mixRenderPass, nullptr);

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
    // Colour round-trips through SHADER_READ_ONLY_OPTIMAL (not GENERAL) between frames - GENERAL
    // disables compression/optimisation many drivers apply to attachment/sampled images, and this
    // target is sampled by the mix pass and/or the final composite pass every single frame.
    // initialLayout must be exactly SHADER_READ_ONLY_OPTIMAL (not UNDEFINED) since LOAD_OP_LOAD
    // requires existing contents (the dirty-rectangle redraw scheme) to be preserved, which an
    // UNDEFINED initial layout would not guarantee - see CreateOffscreenTargets' one-time initial
    // transition, which establishes this invariant before the first frame.
    colourAttachment.initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    colourAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // storeOp = STORE (unlike a typical depth buffer that could discard once used) because the
    // depth-peeling transparency composite pass (applytransparency_vk.frag) samples this depth
    // buffer after the opaque render pass has ended - see uOpaqueDepth.
    // EXPERIMENT: kept at permanent GENERAL (unlike colour) instead of round-tripping through
    // DEPTH_STENCIL_ATTACHMENT_OPTIMAL/SHADER_READ_ONLY_OPTIMAL - profiling showed the opaque
    // pass costing ~8.5x more per-instance in Vulkan than the equivalent OpenGL draw for the same
    // scene, and depth compression metadata (Hi-Z/HTILE-style) tied to attachment-optimal layouts
    // requiring a decompress on every transition to SHADER_READ_ONLY_OPTIMAL is the leading
    // suspect - testing whether reverting just this part closes the gap.
    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = _depthFormat;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_GENERAL;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkAttachmentReference colourRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
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

    // Transparent (depth-peeling) render pass: both colour and depth are CLEARED every single
    // iteration of the peeling loop (unlike the opaque pass's LOAD_OP_LOAD), and both need
    // storeOp = STORE since the depth is read back as next iteration's peeling reference and
    // both colour+depth are sampled by the apply-transparency composite pass. Depth clears to
    // 0.0 (not 1.0 like opaque) to suit the GREATER compare op used for peeling - see
    // rect_vk.frag/VulkanTransparencyDepth.h for why.
    VkAttachmentDescription transparentColourAttachment = colourAttachment;
    transparentColourAttachment.format = VK_FORMAT_R16_UINT;
    transparentColourAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    // Unlike the opaque/mix colour targets, this one has no cross-use content to preserve (it's
    // fully overwritten by loadOp=CLEAR every single peeling iteration), so UNDEFINED is fine and
    // slightly cheaper than requiring it to already be in SHADER_READ_ONLY_OPTIMAL.
    transparentColourAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VkAttachmentDescription transparentDepthAttachment = depthAttachment;

    VkAttachmentDescription transparentAttachments[] = { transparentColourAttachment, transparentDepthAttachment };
    VkRenderPassCreateInfo transparentRenderPassInfo{};
    transparentRenderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    transparentRenderPassInfo.attachmentCount = 2;
    transparentRenderPassInfo.pAttachments = transparentAttachments;
    transparentRenderPassInfo.subpassCount = 1;
    transparentRenderPassInfo.pSubpasses = &subpass;
    transparentRenderPassInfo.dependencyCount = 1;
    transparentRenderPassInfo.pDependencies = &dependency;
    CheckVk(
        vkCreateRenderPass(_device, &transparentRenderPassInfo, nullptr, &_transparentRenderPass),
        "vkCreateRenderPass(transparent)");

    // Mix render pass: colour-only fullscreen composite target (same R8_UINT format as the
    // opaque/mix ping-pong colour images) - loadOp DONT_CARE since the apply-transparency
    // fullscreen triangle unconditionally overwrites every pixel.
    VkAttachmentDescription mixColourAttachment = colourAttachment;
    mixColourAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;

    VkAttachmentReference mixColourRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    VkSubpassDescription mixSubpass{};
    mixSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    mixSubpass.colorAttachmentCount = 1;
    mixSubpass.pColorAttachments = &mixColourRef;

    VkSubpassDependency mixDependency{};
    mixDependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    mixDependency.dstSubpass = 0;
    mixDependency.srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    mixDependency.srcAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    mixDependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    mixDependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo mixRenderPassInfo{};
    mixRenderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    mixRenderPassInfo.attachmentCount = 1;
    mixRenderPassInfo.pAttachments = &mixColourAttachment;
    mixRenderPassInfo.subpassCount = 1;
    mixRenderPassInfo.pSubpasses = &mixSubpass;
    mixRenderPassInfo.dependencyCount = 1;
    mixRenderPassInfo.pDependencies = &mixDependency;
    CheckVk(vkCreateRenderPass(_device, &mixRenderPassInfo, nullptr, &_mixRenderPass), "vkCreateRenderPass(mix)");

    // Rect pipeline layout: push constant (screen size + peeling flag, used by both the opaque
    // and transparent pipeline variants) + atlas/palette/peeling descriptor set.
    VkPushConstantRange pushConstant{};
    pushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstant.offset = 0;
    pushConstant.size = sizeof(float) * 2 + sizeof(int32_t);

    VkPipelineLayoutCreateInfo rectLayoutInfo{};
    rectLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    rectLayoutInfo.setLayoutCount = 1;
    rectLayoutInfo.pSetLayouts = &_rectDescriptorSetLayout;
    rectLayoutInfo.pushConstantRangeCount = 1;
    rectLayoutInfo.pPushConstantRanges = &pushConstant;
    CheckVk(vkCreatePipelineLayout(_device, &rectLayoutInfo, nullptr, &_rectPipelineLayout), "vkCreatePipelineLayout(rect)");

    VkPushConstantRange linePushConstant{};
    linePushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    linePushConstant.offset = 0;
    linePushConstant.size = sizeof(float) * 2;

    VkPipelineLayoutCreateInfo lineLayoutInfo{};
    lineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    lineLayoutInfo.pushConstantRangeCount = 1;
    lineLayoutInfo.pPushConstantRanges = &linePushConstant;
    CheckVk(vkCreatePipelineLayout(_device, &lineLayoutInfo, nullptr, &_linePipelineLayout), "vkCreatePipelineLayout(line)");

    // Apply-transparency composite pipeline layout: no push constants, just the 6-binding
    // descriptor set.
    VkPipelineLayoutCreateInfo transparencyLayoutInfo{};
    transparencyLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    transparencyLayoutInfo.setLayoutCount = 1;
    transparencyLayoutInfo.pSetLayouts = &_transparencyDescriptorSetLayout;
    CheckVk(
        vkCreatePipelineLayout(_device, &transparencyLayoutInfo, nullptr, &_transparencyPipelineLayout),
        "vkCreatePipelineLayout(transparency)");

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

        // Transparent (depth-peeling) variant of the same pipeline: identical shaders/vertex
        // input/layout, just GREATER depth compare (vs opaque's LESS) and targeting the
        // transparent render pass - see VulkanTransparencyDepth.h / rect_vk.frag's uPeeling
        // discard test for why.
        depthStencil.depthCompareOp = VK_COMPARE_OP_GREATER;
        pipelineInfo.renderPass = _transparentRenderPass;
        CheckVk(
            vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &_rectPipelineTransparent),
            "vkCreateGraphicsPipelines(rect transparent)");

        vkDestroyShaderModule(_device, vertModule, nullptr);
        vkDestroyShaderModule(_device, fragModule, nullptr);
    }

    // --- Apply-transparency composite pipeline (fullscreen triangle, no vertex input) ---
    {
        auto vertCode = ReadSpirV("applytransparency_vk.vert.spv");
        auto fragCode = ReadSpirV("applytransparency_vk.frag.spv");
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

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_FALSE;
        depthStencil.depthWriteEnable = VK_FALSE;

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
        pipelineInfo.layout = _transparencyPipelineLayout;
        pipelineInfo.renderPass = _mixRenderPass;
        pipelineInfo.subpass = 0;

        CheckVk(
            vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &_transparencyPipeline),
            "vkCreateGraphicsPipelines(transparency)");

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

    auto destroyFramebuffer = [this](VkFramebuffer& fb) {
        if (fb != VK_NULL_HANDLE)
        {
            vkDestroyFramebuffer(_device, fb, nullptr);
            fb = VK_NULL_HANDLE;
        }
    };
    auto destroyImageTarget = [this](ImageTarget& target) {
        if (target.view != VK_NULL_HANDLE)
        {
            vkDestroyImageView(_device, target.view, nullptr);
            target.view = VK_NULL_HANDLE;
        }
        if (target.image != VK_NULL_HANDLE)
        {
            vkDestroyImage(_device, target.image, nullptr);
            target.image = VK_NULL_HANDLE;
        }
        if (target.memory != VK_NULL_HANDLE)
        {
            vkFreeMemory(_device, target.memory, nullptr);
            target.memory = VK_NULL_HANDLE;
        }
    };

    for (auto& fb : _opaqueFramebuffers)
        destroyFramebuffer(fb);
    for (auto& fb : _mixFramebuffers)
        destroyFramebuffer(fb);
    for (auto& fb : _transparentFramebuffers)
        destroyFramebuffer(fb);

    for (auto& target : _colourTargets)
        destroyImageTarget(target);
    destroyImageTarget(_opaqueDepth);
    destroyImageTarget(_transparentColour);
    for (auto& target : _transparentDepthTargets)
        destroyImageTarget(target);
}

void VulkanDrawingContext::CreateOffscreenTargets()
{
    auto createImageTarget = [this](VkFormat format, VkImageUsageFlags usage, VkImageAspectFlags aspect) -> ImageTarget {
        ImageTarget target;

        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = format;
        imageInfo.extent = { _width, _height, 1 };
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = usage;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        CheckVk(vkCreateImage(_device, &imageInfo, nullptr, &target.image), "vkCreateImage(offscreen)");

        VkMemoryRequirements memReq{};
        vkGetImageMemoryRequirements(_device, target.image, &memReq);
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReq.size;
        alloc.memoryTypeIndex = FindMemoryType(_physicalDevice, memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        CheckVk(vkAllocateMemory(_device, &alloc, nullptr, &target.memory), "vkAllocateMemory(offscreen)");
        CheckVk(vkBindImageMemory(_device, target.image, target.memory, 0), "vkBindImageMemory(offscreen)");

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = target.image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspect;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        CheckVk(vkCreateImageView(_device, &viewInfo, nullptr, &target.view), "vkCreateImageView(offscreen)");

        return target;
    };

    constexpr VkImageUsageFlags kColourUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    constexpr VkImageUsageFlags kDepthUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

    for (auto& target : _colourTargets)
        target = createImageTarget(VK_FORMAT_R8_UINT, kColourUsage, VK_IMAGE_ASPECT_COLOR_BIT);
    _opaqueDepth = createImageTarget(_depthFormat, kDepthUsage, VK_IMAGE_ASPECT_DEPTH_BIT);
    _transparentColour = createImageTarget(VK_FORMAT_R16_UINT, kColourUsage, VK_IMAGE_ASPECT_COLOR_BIT);
    for (auto& target : _transparentDepthTargets)
        target = createImageTarget(_depthFormat, kDepthUsage, VK_IMAGE_ASPECT_DEPTH_BIT);

    for (uint32_t i = 0; i < 2; i++)
    {
        VkImageView opaqueAttachments[] = { _colourTargets[i].view, _opaqueDepth.view };
        VkFramebufferCreateInfo opaqueFbInfo{};
        opaqueFbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        opaqueFbInfo.renderPass = _offscreenRenderPass;
        opaqueFbInfo.attachmentCount = 2;
        opaqueFbInfo.pAttachments = opaqueAttachments;
        opaqueFbInfo.width = _width;
        opaqueFbInfo.height = _height;
        opaqueFbInfo.layers = 1;
        CheckVk(
            vkCreateFramebuffer(_device, &opaqueFbInfo, nullptr, &_opaqueFramebuffers[i]), "vkCreateFramebuffer(opaque)");

        VkFramebufferCreateInfo mixFbInfo{};
        mixFbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        mixFbInfo.renderPass = _mixRenderPass;
        mixFbInfo.attachmentCount = 1;
        mixFbInfo.pAttachments = &_colourTargets[i].view;
        mixFbInfo.width = _width;
        mixFbInfo.height = _height;
        mixFbInfo.layers = 1;
        CheckVk(vkCreateFramebuffer(_device, &mixFbInfo, nullptr, &_mixFramebuffers[i]), "vkCreateFramebuffer(mix)");

        VkImageView transparentAttachments[] = { _transparentColour.view, _transparentDepthTargets[i].view };
        VkFramebufferCreateInfo transparentFbInfo{};
        transparentFbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        transparentFbInfo.renderPass = _transparentRenderPass;
        transparentFbInfo.attachmentCount = 2;
        transparentFbInfo.pAttachments = transparentAttachments;
        transparentFbInfo.width = _width;
        transparentFbInfo.height = _height;
        transparentFbInfo.layers = 1;
        CheckVk(
            vkCreateFramebuffer(_device, &transparentFbInfo, nullptr, &_transparentFramebuffers[i]),
            "vkCreateFramebuffer(transparent)");
    }

    _currentColourIndex = 0;

    // One-time initial clear of the colour targets to palette index 0 (transparent), mirroring
    // OpenGLDrawingEngine::Resize()'s explicit Clear() call - subsequent frames rely on
    // LOAD_OP_LOAD and only redraw dirty regions. This also establishes the colour targets'
    // steady-state resting layout (SHADER_READ_ONLY_OPTIMAL, matching every render pass's
    // initialLayout/finalLayout for these attachments - see CreatePipelines) before the first
    // frame runs. The depth/transparent-colour targets (EXPERIMENT: depth kept at permanent
    // GENERAL, see CreatePipelines) are transitioned below too; _transparentColour needs no
    // initial transition since its render pass always declares initialLayout = UNDEFINED (safe
    // because it's unconditionally cleared - LOAD_OP_CLEAR - on every single use).
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

    VkClearColorValue clearColour{};
    clearColour.uint32[0] = 0;
    VkImageSubresourceRange colourRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };

    for (auto& target : _colourTargets)
    {
        // vkCmdClearColorImage only accepts GENERAL or TRANSFER_DST_OPTIMAL, so transition
        // through TRANSFER_DST_OPTIMAL and then land on the steady-state SHADER_READ_ONLY_OPTIMAL
        // that every render pass using this attachment expects to find it in.
        TransitionImageLayout(
            cmd, target.image, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        vkCmdClearColorImage(cmd, target.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearColour, 1, &colourRange);
        TransitionImageLayout(
            cmd, target.image, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    // EXPERIMENT: opaque/transparent depth targets kept at permanent GENERAL (see
    // CreatePipelines) rather than relying on initialLayout = UNDEFINED, so they need an explicit
    // one-time transition here too.
    TransitionImageLayout(cmd, _opaqueDepth.image, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    for (auto& target : _transparentDepthTargets)
    {
        TransitionImageLayout(cmd, target.image, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    }

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

    // The offscreen colour/depth targets these caches reference are all destroyed and recreated
    // (new VkImageView handles) above - reset to the "unwritten" sentinel so the next
    // FlushRectangles/HandleTransparency call always rewrites them instead of potentially (if a
    // freed handle happened to be reused) skipping a write that's actually needed. The atlas and
    // palette/blend-palette bindings are unaffected by resize and don't need resetting here.
    _rectPeelingBoundView.fill(VK_NULL_HANDLE);
    _compositeOpaqueColourBoundView.fill(VK_NULL_HANDLE);
    _compositeOpaqueDepthBoundView.fill(VK_NULL_HANDLE);
    _compositeTransparentColourBoundView.fill(VK_NULL_HANDLE);
    _compositeTransparentDepthBoundView.fill(VK_NULL_HANDLE);

    _rects.clear();
    _transparentRects.clear();
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
    // Grow to 1.5x the requested size (like a typical vector growth policy), not an exact fit -
    // an exact-fit policy would otherwise force a full buffer destroy+recreate+remap on every
    // single frame whenever the draw-call count keeps slowly climbing (e.g. while panning across
    // an increasingly busy part of the park), instead of only occasionally as the buffer grows
    // in front of demand.
    VkDeviceSize newSize = std::max<VkDeviceSize>(requiredSize + requiredSize / 2, 1 << 14);
    buffer = CreateBuffer(
        _device, _physicalDevice, newSize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
}

void VulkanDrawingContext::FlushCommandBuffers(VkCommandBuffer cmd, uint32_t frameIndex)
{
    Guard::Assert(_inDraw == true);

    if (_opaqueFramebuffers[0] == VK_NULL_HANDLE)
    {
        _rects.clear();
        _transparentRects.clear();
        _lines.clear();
        return;
    }

    // Upload any newly-seen sprite/glyph/text texture data before the render pass begins.
    // FlushPendingUploads() is a no-op (issues no commands at all) on the very common case of a
    // frame with no newly-seen textures; when it does have uploads, it already transitions the
    // atlas to/from VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL around them with proper barriers, so no
    // additional barrier is needed here.
    _textureCache.FlushPendingUploads(cmd, _stagingBuffers[frameIndex]);

    VkClearValue depthClear{};
    depthClear.depthStencil = { 1.0f, 0 };
    VkClearValue clearValues[] = { {}, depthClear };

    VkRenderPassBeginInfo rpBegin{};
    rpBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBegin.renderPass = _offscreenRenderPass;
    rpBegin.framebuffer = _opaqueFramebuffers[_currentColourIndex];
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

    if (_transparentRects.size() > 0)
    {
        HandleTransparency(cmd, frameIndex);
        _transparentRects.clear();
    }
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

void VulkanDrawingContext::UpdateRectDescriptorSetIfChanged(uint32_t frameIndex, VkImageView peelingView)
{
    VkDescriptorImageInfo atlasInfo{};
    atlasInfo.sampler = _atlasSampler;
    atlasInfo.imageView = _textureCache.GetAtlasImageView();
    atlasInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo paletteInfo{};
    paletteInfo.sampler = _paletteSampler;
    paletteInfo.imageView = _textureCache.GetPaletteImageView();
    paletteInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo peelingInfo{};
    peelingInfo.sampler = _paletteSampler;
    peelingInfo.imageView = peelingView;
    // EXPERIMENT: peelingView is always one of _opaqueDepth/_transparentDepthTargets, which are
    // being kept at permanent GENERAL for this test (see CreatePipelines).
    peelingInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkWriteDescriptorSet writes[3]{};
    uint32_t writeCount = 0;

    uint64_t atlasVersion = _textureCache.GetAtlasVersion();
    if (_rectAtlasBoundVersion[frameIndex] != atlasVersion)
    {
        writes[writeCount] = {};
        writes[writeCount].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[writeCount].dstSet = _rectDescriptorSets[frameIndex];
        writes[writeCount].dstBinding = 0;
        writes[writeCount].descriptorCount = 1;
        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[writeCount].pImageInfo = &atlasInfo;
        writeCount++;
        _rectAtlasBoundVersion[frameIndex] = atlasVersion;
    }

    if (_rectPaletteBoundView[frameIndex] != paletteInfo.imageView)
    {
        writes[writeCount] = {};
        writes[writeCount].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[writeCount].dstSet = _rectDescriptorSets[frameIndex];
        writes[writeCount].dstBinding = 1;
        writes[writeCount].descriptorCount = 1;
        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[writeCount].pImageInfo = &paletteInfo;
        writeCount++;
        _rectPaletteBoundView[frameIndex] = paletteInfo.imageView;
    }

    if (_rectPeelingBoundView[frameIndex] != peelingView)
    {
        writes[writeCount] = {};
        writes[writeCount].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[writeCount].dstSet = _rectDescriptorSets[frameIndex];
        writes[writeCount].dstBinding = 2;
        writes[writeCount].descriptorCount = 1;
        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[writeCount].pImageInfo = &peelingInfo;
        writeCount++;
        _rectPeelingBoundView[frameIndex] = peelingView;
    }

    if (writeCount > 0)
        vkUpdateDescriptorSets(_device, writeCount, writes, 0, nullptr);
}

void VulkanDrawingContext::FlushRectangles(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (_rects.size() == 0)
        return;

    VkDeviceSize requiredSize = _rects.size() * sizeof(VulkanDrawRectCommand);
    EnsureInstanceBufferCapacity(_rectInstanceBuffers[frameIndex], requiredSize);
    std::memcpy(_rectInstanceBuffers[frameIndex].mapped, _rects.data(), requiredSize);

    // Binding 2 (peeling-reference depth) is unused by the opaque pipeline (uPeeling=0 causes
    // rect_vk.frag to skip sampling it entirely) but the descriptor set still needs a valid,
    // compatible image bound to satisfy validation - the opaque depth view is a harmless filler.
    UpdateRectDescriptorSetIfChanged(frameIndex, _opaqueDepth.view);

    struct
    {
        float screenSize[2];
        int32_t peeling;
    } pushConstants{ { static_cast<float>(_width), static_cast<float>(_height) }, 0 };

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipeline);
    vkCmdPushConstants(
        cmd, _rectPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pushConstants),
        &pushConstants);
    vkCmdBindDescriptorSets(
        cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipelineLayout, 0, 1, &_rectDescriptorSets[frameIndex], 0, nullptr);

    VkBuffer vbo = _rectInstanceBuffers[frameIndex].buffer;
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &offset);
    vkCmdDraw(cmd, 4, static_cast<uint32_t>(_rects.size()), 0, 0);

    _rects.clear();
}

void VulkanDrawingContext::UpdateCompositeDescriptorSetIfChanged(
    uint32_t frameIndex, VkImageView opaqueColourView, VkImageView opaqueDepthView, VkImageView transparentColourView,
    VkImageView transparentDepthView)
{
    VkImageView views[6] = {
        opaqueColourView,
        opaqueDepthView,
        transparentColourView,
        transparentDepthView,
        _textureCache.GetPaletteImageView(),
        _textureCache.GetBlendPaletteImageView(),
    };
    std::array<VkImageView, kVulkanFramesInFlight>* caches[6] = {
        &_compositeOpaqueColourBoundView,     &_compositeOpaqueDepthBoundView, &_compositeTransparentColourBoundView,
        &_compositeTransparentDepthBoundView, &_compositePaletteBoundView,     &_compositeBlendPaletteBoundView,
    };
    // Bindings 0-3 sample the ping-ponged offscreen colour/depth targets. Binding 0 (opaque
    // colour) and 2 (transparent colour) are SHADER_READ_ONLY_OPTIMAL (see CreatePipelines);
    // bindings 1/3 (opaque/transparent depth) are EXPERIMENTALLY kept at permanent GENERAL (see
    // CreatePipelines) rather than SHADER_READ_ONLY_OPTIMAL, to test whether depth compression
    // decompress overhead explains the measured Vulkan-vs-OpenGL opaque-pass gap. Bindings 4-5
    // are the palette/blend-palette lookup textures (also SHADER_READ_ONLY_OPTIMAL, immutable
    // after their one-time creation).
    VkImageLayout layouts[6] = { VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

    VkDescriptorImageInfo infos[6]{};
    VkWriteDescriptorSet writes[6]{};
    uint32_t writeCount = 0;

    for (uint32_t b = 0; b < 6; b++)
    {
        if ((*caches[b])[frameIndex] == views[b])
            continue;

        infos[writeCount].sampler = _paletteSampler;
        infos[writeCount].imageView = views[b];
        infos[writeCount].imageLayout = layouts[b];

        writes[writeCount].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[writeCount].dstSet = _transparencyDescriptorSets[frameIndex];
        writes[writeCount].dstBinding = b;
        writes[writeCount].descriptorCount = 1;
        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[writeCount].pImageInfo = &infos[writeCount];
        writeCount++;

        (*caches[b])[frameIndex] = views[b];
    }

    if (writeCount > 0)
        vkUpdateDescriptorSets(_device, writeCount, writes, 0, nullptr);
}

// Depth-peeling transparency, verbatim port of the OpenGL renderer's
// SwapFramebuffer/HandleTransparency architecture (see VulkanDrawingContext.h class comment and
// VulkanTransparencyDepth.h). Called once per frame, after the opaque render pass has ended,
// only when there is at least one transparent draw call queued.
void VulkanDrawingContext::HandleTransparency(VkCommandBuffer cmd, uint32_t frameIndex)
{
    int32_t maxDepth = MaxTransparencyDepth(_transparentRects);
    if (maxDepth <= 0)
        return;

    // The transparent batch is redrawn, unchanged, on every peeling iteration - only the
    // peeling-reference depth texture and comparison changes which fragments survive each time.
    VkDeviceSize requiredSize = _transparentRects.size() * sizeof(VulkanDrawRectCommand);
    EnsureInstanceBufferCapacity(_transparentRectInstanceBuffers[frameIndex], requiredSize);
    std::memcpy(_transparentRectInstanceBuffers[frameIndex].mapped, _transparentRects.data(), requiredSize);

    VkViewport viewport{};
    viewport.width = static_cast<float>(_width);
    viewport.height = static_cast<float>(_height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{};
    scissor.extent = { _width, _height };

    VkBuffer vbo = _transparentRectInstanceBuffers[frameIndex].buffer;
    VkDeviceSize vboOffset = 0;

    // Viewport/scissor are identical for every iteration and every pass below (always the full
    // offscreen target extent) - dynamic state set via vkCmdSetViewport/vkCmdSetScissor persists
    // across render pass boundaries within the same command buffer, so set them once here instead
    // of redundantly inside the loop.
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    for (int32_t i = 0; i < maxDepth; i++)
    {
        uint32_t frontDepthIdx = static_cast<uint32_t>(i % 2);
        uint32_t backDepthIdx = static_cast<uint32_t>((i + 1) % 2);

        // --- Transparent (depth-peeling) pass: draw the transparent batch, discarding any
        // fragment that isn't strictly farther than the previous iteration's surviving depth
        // (see rect_vk.frag's uPeeling discard test) - this progressively exposes deeper
        // overlapping transparent layers on each iteration. ---
        VkClearValue transparentClearValues[2]{};
        transparentClearValues[0].color.uint32[0] = 0;
        transparentClearValues[1].depthStencil = { 0.0f, 0 };

        VkRenderPassBeginInfo transparentBegin{};
        transparentBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        transparentBegin.renderPass = _transparentRenderPass;
        transparentBegin.framebuffer = _transparentFramebuffers[frontDepthIdx];
        transparentBegin.renderArea.extent = { _width, _height };
        transparentBegin.clearValueCount = 2;
        transparentBegin.pClearValues = transparentClearValues;
        vkCmdBeginRenderPass(cmd, &transparentBegin, VK_SUBPASS_CONTENTS_INLINE);

        VkImageView peelingView = i > 0 ? _transparentDepthTargets[backDepthIdx].view : _opaqueDepth.view;
        UpdateRectDescriptorSetIfChanged(frameIndex, peelingView);

        struct
        {
            float screenSize[2];
            int32_t peeling;
        } pushConstants{ { static_cast<float>(_width), static_cast<float>(_height) }, i > 0 ? 1 : 0 };

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipelineTransparent);
        vkCmdPushConstants(
            cmd, _rectPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pushConstants),
            &pushConstants);
        vkCmdBindDescriptorSets(
            cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _rectPipelineLayout, 0, 1, &_rectDescriptorSets[frameIndex], 0, nullptr);

        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &vboOffset);
        vkCmdDraw(cmd, 4, static_cast<uint32_t>(_transparentRects.size()), 0, 0);

        vkCmdEndRenderPass(cmd);

        // --- Mix composite pass: blend the freshly-peeled transparent layer against the
        // current opaque image into the "other" colour target, matching
        // SwapFramebuffer::ApplyTransparency - see applytransparency_vk.frag. ---
        uint32_t mixIdx = 1 - _currentColourIndex;

        VkRenderPassBeginInfo mixBegin{};
        mixBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        mixBegin.renderPass = _mixRenderPass;
        mixBegin.framebuffer = _mixFramebuffers[mixIdx];
        mixBegin.renderArea.extent = { _width, _height };
        vkCmdBeginRenderPass(cmd, &mixBegin, VK_SUBPASS_CONTENTS_INLINE);

        UpdateCompositeDescriptorSetIfChanged(
            frameIndex, _colourTargets[_currentColourIndex].view, _opaqueDepth.view, _transparentColour.view,
            _transparentDepthTargets[frontDepthIdx].view);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _transparencyPipeline);
        vkCmdBindDescriptorSets(
            cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _transparencyPipelineLayout, 0, 1,
            &_transparencyDescriptorSets[frameIndex], 0, nullptr);
        vkCmdDraw(cmd, 3, 1, 0, 0);

        vkCmdEndRenderPass(cmd);

        _currentColourIndex = mixIdx;
    }
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

    // Routed into the depth-peeling transparent batch (matches OpenGL's
    // OpenGLDrawingContext::FilterRect - see VulkanDrawingContext.h class comment).
    VulkanDrawRectCommand& command = _transparentRects.allocate();
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

    // Blended/water sprites are routed into the depth-peeling transparent batch, matching
    // OpenGL's OpenGLDrawingContext::DrawSprite - see VulkanDrawingContext.h class comment.
    if (special || imageId.IsBlended())
    {
        VulkanDrawRectCommand& command = _transparentRects.allocate();
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
