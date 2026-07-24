/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <vulkan/vulkan.h>

#include <stdexcept>
#include <string>

namespace OpenRCT2::Ui
{
    inline void CheckVk(VkResult result, const char* what)
    {
        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(std::string(what) + " failed with error code " + std::to_string(result));
        }
    }

    inline uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties)
    {
        VkPhysicalDeviceMemoryProperties memProperties{};
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
        {
            if ((typeFilter & (1u << i)) != 0 && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("Unable to find suitable Vulkan memory type.");
    }

    struct VulkanBuffer
    {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkDeviceSize size = 0;
        void* mapped = nullptr;
    };

    inline VulkanBuffer CreateBuffer(
        VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize size, VkBufferUsageFlags usage,
        VkMemoryPropertyFlags properties)
    {
        VulkanBuffer result{};
        result.size = size;

        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        CheckVk(vkCreateBuffer(device, &bufferInfo, nullptr, &result.buffer), "vkCreateBuffer");

        VkMemoryRequirements memReq{};
        vkGetBufferMemoryRequirements(device, result.buffer, &memReq);

        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReq.size;
        alloc.memoryTypeIndex = FindMemoryType(physicalDevice, memReq.memoryTypeBits, properties);
        CheckVk(vkAllocateMemory(device, &alloc, nullptr, &result.memory), "vkAllocateMemory(buffer)");
        CheckVk(vkBindBufferMemory(device, result.buffer, result.memory, 0), "vkBindBufferMemory");

        if ((properties & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
        {
            CheckVk(vkMapMemory(device, result.memory, 0, size, 0, &result.mapped), "vkMapMemory(buffer)");
        }

        return result;
    }

    inline void DestroyBuffer(VkDevice device, VulkanBuffer& buffer)
    {
        if (buffer.buffer != VK_NULL_HANDLE)
        {
            if (buffer.mapped != nullptr)
            {
                vkUnmapMemory(device, buffer.memory);
                buffer.mapped = nullptr;
            }
            vkDestroyBuffer(device, buffer.buffer, nullptr);
            buffer.buffer = VK_NULL_HANDLE;
        }
        if (buffer.memory != VK_NULL_HANDLE)
        {
            vkFreeMemory(device, buffer.memory, nullptr);
            buffer.memory = VK_NULL_HANDLE;
        }
    }

    // General purpose image layout transition, sufficient for the layouts used throughout the
    // Vulkan renderer (transfer src/dst, shader read-only, colour/depth attachment, present).
    inline void TransitionImageLayout(
        VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect, VkImageLayout oldLayout, VkImageLayout newLayout,
        uint32_t layerCount = 1, uint32_t mipLevels = 1)
    {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = aspect;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = mipLevels;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = layerCount;

        VkPipelineStageFlags srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = 0;

        switch (oldLayout)
        {
            case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
                srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                break;
            case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
                srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                break;
            case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
                srcStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
                barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
                break;
            case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
                srcStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                break;
            case VK_IMAGE_LAYOUT_PRESENT_SRC_KHR:
                srcStage = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
                break;
            default:
                break;
        }

        switch (newLayout)
        {
            case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
                dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                break;
            case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
                dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                break;
            case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
                dstStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                break;
            case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
                dstStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                break;
            case VK_IMAGE_LAYOUT_PRESENT_SRC_KHR:
                dstStage = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
                break;
            default:
                break;
        }

        vkCmdPipelineBarrier(cmd, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    }
} // namespace OpenRCT2::Ui
