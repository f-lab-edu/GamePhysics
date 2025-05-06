//
//  OffscreenRenderer.h
//
#pragma once
#include <vulkan/vulkan.h>

class DeviceContext;
class Buffer;
struct RenderModel;

bool InitOffscreen( DeviceContext * device, int width, int height );
bool CleanupOffscreen( DeviceContext * device );
void DrawDebugAxes(VkCommandBuffer cmdBuffer, DeviceContext* device, const RenderModel& model, Buffer* uniforms);
void DrawOffscreen( DeviceContext * device, int cmdBufferIndex, Buffer * uniforms, const RenderModel * renderModels, const int numModels, bool isDebug );
