//
//  OffscreenRenderer.cpp
//
#include "OffscreenRenderer.h"
#include "model.h"
#include "Samplers.h"

#include "../application.h"
#include <assert.h>
#include <stdio.h>
#include <vector>

FrameBuffer	g_offscreenFrameBuffer;
Pipeline	g_skyPipeline;
Shader		g_skyShader;
Descriptors	g_skyDescriptors;
Model		g_skyModel;

Pipeline	g_checkerboardShadowPipeline;
Shader		g_checkerboardShadowShader;
Descriptors	g_checkerboardShadowDescriptors;

FrameBuffer	g_shadowFrameBuffer;
Pipeline	g_shadowPipeline;
Shader		g_shadowShader;
Descriptors	g_shadowDescriptors;

/*
====================================================
InitOffscreen
====================================================
*/
bool InitOffscreen( DeviceContext * device, int width, int height ) {
	bool result;

	//
	//	Build the frame buffer to render into
	//
	{
		FrameBuffer::CreateParms_t frameBufferParms;
		frameBufferParms.width = width;
		frameBufferParms.height = height;
		frameBufferParms.hasColor = true;
		frameBufferParms.hasDepth = true;
		result = g_offscreenFrameBuffer.Create( device, frameBufferParms );
		if ( !result ) {
			printf( "ERROR: Failed to create off screen buffer\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Shadow
	//
	{
		FrameBuffer::CreateParms_t frameBufferParms;
		frameBufferParms.width = 4096;
		frameBufferParms.height = 4096;
		frameBufferParms.hasColor = false;
		frameBufferParms.hasDepth = true;
		result = g_shadowFrameBuffer.Create( device, frameBufferParms );
		if ( !result ) {
			printf( "ERROR: Failed to create off screen buffer\n" );
			assert( 0 );
			return false;
		}

		result = g_shadowShader.Load( device, "shadow2" );
		if ( !result ) {
			printf( "ERROR: Failed to load shader\n" );
			assert( 0 );
			return false;
		}

		Descriptors::CreateParms_t descriptorParms;
		memset( &descriptorParms, 0, sizeof( descriptorParms ) );
		descriptorParms.numUniformsVertex = 2;
		result = g_shadowDescriptors.Create( device, descriptorParms );
		if ( !result ) {
			printf( "ERROR: Failed to build descriptors\n" );
			assert( 0 );
			return false;
		}

		Pipeline::CreateParms_t pipelineParms;
		pipelineParms.framebuffer = &g_shadowFrameBuffer;
		pipelineParms.descriptors = &g_shadowDescriptors;
		pipelineParms.shader = &g_shadowShader;
		pipelineParms.width = frameBufferParms.width;
		pipelineParms.height = frameBufferParms.height;
		pipelineParms.cullMode = Pipeline::CULL_MODE_FRONT;
		pipelineParms.depthTest = true;
		pipelineParms.depthWrite = true;
		result = g_shadowPipeline.Create( device, pipelineParms );
		if ( !result ) {
			printf( "ERROR: Failed to build pipeline\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Sky
	//
	{
		result = g_skyShader.Load( device, "sky" );
		if ( !result ) {
			printf( "ERROR: Failed to load shader\n" );
			assert( 0 );
			return false;
		}

		Descriptors::CreateParms_t descriptorParms;
		memset( &descriptorParms, 0, sizeof( descriptorParms ) );
		descriptorParms.numUniformsVertex = 1;
		result = g_skyDescriptors.Create( device, descriptorParms );
		if ( !result ) {
			printf( "ERROR: Failed to build descriptors\n" );
			assert( 0 );
			return false;
		}

		Pipeline::CreateParms_t pipelineParms;
		pipelineParms.framebuffer = &g_offscreenFrameBuffer;
		pipelineParms.descriptors = &g_skyDescriptors;
		pipelineParms.shader = &g_skyShader;
		pipelineParms.width = g_offscreenFrameBuffer.m_parms.width;
		pipelineParms.height = g_offscreenFrameBuffer.m_parms.height;
		pipelineParms.cullMode = Pipeline::CULL_MODE_NONE;
		pipelineParms.depthTest = false;
		pipelineParms.depthWrite = false;
		result = g_skyPipeline.Create( device, pipelineParms );
		if ( !result ) {
			printf( "ERROR: Failed to build pipeline\n" );
			assert( 0 );
			return false;
		}

		ShapeSphere sphereShape( 1.0f );
		g_skyModel.BuildFromShape( &sphereShape );
		g_skyModel.MakeVBO( device );
	}

	//
	//	CheckerBoard Shadow
	//
	{
		result = g_checkerboardShadowShader.Load( device, "checkerboardShadowed2" );
		if ( !result ) {
			printf( "ERROR: Failed to load shader\n" );
			assert( 0 );
			return false;
		}

		Descriptors::CreateParms_t descriptorParms;
		memset( &descriptorParms, 0, sizeof( descriptorParms ) );
		descriptorParms.numUniformsVertex = 3;
		descriptorParms.numUniformsFragment = 1;
		descriptorParms.numImageSamplers = 1;
		result = g_checkerboardShadowDescriptors.Create( device, descriptorParms );
		if ( !result ) {
			printf( "ERROR: Failed to build descriptors\n" );
			assert( 0 );
			return false;
		}

		Pipeline::CreateParms_t pipelineParms;
		pipelineParms.framebuffer = &g_offscreenFrameBuffer;
		pipelineParms.descriptors = &g_checkerboardShadowDescriptors;
		pipelineParms.shader = &g_checkerboardShadowShader;
		pipelineParms.width = g_offscreenFrameBuffer.m_parms.width;
		pipelineParms.height = g_offscreenFrameBuffer.m_parms.height;
		pipelineParms.cullMode = Pipeline::CULL_MODE_BACK;
		pipelineParms.depthTest = true;
		pipelineParms.depthWrite = true;
		result = g_checkerboardShadowPipeline.Create( device, pipelineParms );
		if ( !result ) {
			printf( "ERROR: Failed to build pipeline\n" );
			assert( 0 );
			return false;
		}
	}

	return true;
}

/*
====================================================
CleanupOffscreen
====================================================
*/
bool CleanupOffscreen( DeviceContext * device ) {
	g_skyPipeline.Cleanup( device );
	g_skyDescriptors.Cleanup( device );
	g_skyShader.Cleanup( device );
	g_offscreenFrameBuffer.Cleanup( device );
	g_skyModel.Cleanup( *device );

	g_checkerboardShadowPipeline.Cleanup( device );
	g_checkerboardShadowShader.Cleanup( device );
	g_checkerboardShadowDescriptors.Cleanup( device );
	
	g_shadowPipeline.Cleanup( device );
	g_shadowShader.Cleanup( device );
	g_shadowDescriptors.Cleanup( device );
	g_shadowFrameBuffer.Cleanup( device );
	return true;
}


/*
====================================================
DrawDebugAxes
====================================================
*/
void DrawDebugAxes(VkCommandBuffer cmdBuffer, DeviceContext* device, const RenderModel& model, Buffer* uniforms) {
	// Define axis lines (X: red, Y: green, Z: blue) with 100-unit length for visibility
	std::vector<vert_t> debugVertices = {
		// X-axis (red)
		{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {255, 0, 0, 255}},
		{{7.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {255, 0, 0, 255}},
		// Y-axis (green)
		{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 255, 0, 255}},
		{{0.0f, 7.0f, 0.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 255, 0, 255}},
		// Z-axis (blue)
		{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 255, 255}},
		{{0.0f, 0.0f, 7.0f}, {0.0f, 0.0f}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 255, 255}}
	};
	// Define indices for lines
	std::vector<unsigned int> debugIndices = { 0, 1, 2, 3, 4, 5 };

	// Append to model's vertex buffer
	VkDeviceSize debugVertexSize = sizeof(vert_t) * debugVertices.size();
	VkDeviceSize vertexOffset = sizeof(double) * model.model->m_vertices.size();

	void* data;
	vkMapMemory(device->m_vkDevice, model.model->m_vertexBuffer.m_vkBufferMemory, vertexOffset, debugVertexSize, 0, &data);
	memcpy(data, debugVertices.data(), debugVertexSize);
	vkUnmapMemory(device->m_vkDevice, model.model->m_vertexBuffer.m_vkBufferMemory);

	// Append to model's index buffer
	VkDeviceSize debugIndexSize = sizeof(unsigned int) * debugIndices.size();
	VkDeviceSize indexOffset = sizeof(unsigned int) * model.model->m_indices.size();

	vkMapMemory(device->m_vkDevice, model.model->m_indexBuffer.m_vkBufferMemory, indexOffset, debugIndexSize, 0, &data);
	memcpy(data, debugIndices.data(), debugIndexSize);
	vkUnmapMemory(device->m_vkDevice, model.model->m_indexBuffer.m_vkBufferMemory);

	// Bind pipeline
	
	extern Pipeline g_checkerboardShadowPipeline;
	g_checkerboardShadowPipeline.BindPipeline(cmdBuffer);

	// Bind vertex and index buffers
	VkBuffer vertexBuffers[] = { model.model->m_vertexBuffer.m_vkBuffer };
	VkDeviceSize offsets[] = { vertexOffset };
	vkCmdBindVertexBuffers(cmdBuffer, 0, 1, vertexBuffers, offsets);
	vkCmdBindIndexBuffer(cmdBuffer, model.model->m_indexBuffer.m_vkBuffer, indexOffset, VK_INDEX_TYPE_UINT32);

	// Bind descriptor
	Descriptor descriptor = g_checkerboardShadowPipeline.GetFreeDescriptor();
	const int camOffset = 0;
	const int camSize = sizeof(float) * 16 * 4;
	const int shadowCamOffset = device->GetAligendUniformByteOffset(camOffset + camSize);
	const int shadowCamSize = camSize;
	descriptor.BindBuffer(uniforms, camOffset, camSize, 0);
	descriptor.BindBuffer(uniforms, model.uboByteOffset, model.uboByteSize, 1);
	descriptor.BindBuffer(uniforms, shadowCamOffset, shadowCamSize, 2);
	extern FrameBuffer g_shadowFrameBuffer;
	descriptor.BindImage(VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
		g_shadowFrameBuffer.m_imageDepth.m_vkImageView,
		Samplers::m_samplerStandard, 0);
	descriptor.BindDescriptor(device, cmdBuffer, &g_checkerboardShadowPipeline);

	// Draw indexed
	vkCmdDrawIndexed(cmdBuffer, debugIndices.size(), 1, 0, 0, 0);
}

/*
====================================================
DrawOffscreen
====================================================
*/
void DrawOffscreen( DeviceContext * device, int cmdBufferIndex, Buffer * uniforms, const RenderModel * renderModels, const int numModels, bool isDebug) {
	VkCommandBuffer cmdBuffer = device->m_vkCommandBuffers[ cmdBufferIndex ];

	const int camOffset = 0;
	const int camSize = sizeof( float ) * 16 * 4;

	const int shadowCamOffset = device->GetAligendUniformByteOffset( camOffset + camSize );
	const int shadowCamSize = camSize;

	//
	//	Update the Shadows
	//
	{
		g_shadowFrameBuffer.m_imageDepth.TransitionLayout( cmdBuffer, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL );

		g_shadowFrameBuffer.BeginRenderPass( device, cmdBufferIndex );

		// Binding the pipeline is effectively the "use shader" we had back in our opengl apps
		g_shadowPipeline.BindPipeline( cmdBuffer );
		for ( int i = 0; i < numModels; i++ ) {
			const RenderModel & renderModel = renderModels[ i ];

			// Descriptor is how we bind our buffers and images
			Descriptor descriptor = g_shadowPipeline.GetFreeDescriptor();
			descriptor.BindBuffer( uniforms, shadowCamOffset, shadowCamSize, 0 );						// bind the camera matrices
			descriptor.BindBuffer( uniforms, renderModel.uboByteOffset, renderModel.uboByteSize, 1 );	// bind the model matrices
			descriptor.BindDescriptor( device, cmdBuffer, &g_shadowPipeline );
			renderModel.model->DrawIndexed( cmdBuffer );
		}

		g_shadowFrameBuffer.EndRenderPass( device, cmdBufferIndex );

		g_shadowFrameBuffer.m_imageDepth.TransitionLayout( cmdBuffer, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL );
	}

	//
	//	Draw the World
	//
	{
		g_offscreenFrameBuffer.m_imageColor.TransitionLayout( cmdBuffer, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );

		g_offscreenFrameBuffer.BeginRenderPass( device, cmdBufferIndex );

		//
		//	Draw the sky
		//
		{
			// Binding the pipeline is effectively the "use shader" we had back in our opengl apps
			g_skyPipeline.BindPipeline( cmdBuffer );
	
			// Descriptor is how we bind our buffers and images
			Descriptor descriptor = g_skyPipeline.GetFreeDescriptor();
			descriptor.BindBuffer( uniforms, camOffset, camSize, 0 );
			descriptor.BindDescriptor( device, cmdBuffer, &g_skyPipeline );
			g_skyModel.DrawIndexed( cmdBuffer );
		}
	
		//
		//	Draw the models
		//
		{
			// Binding the pipeline is effectively the "use shader" we had back in our opengl apps
			g_checkerboardShadowPipeline.BindPipeline( cmdBuffer );
			for ( int i = 0; i < numModels; i++ ) {
				const RenderModel & renderModel = renderModels[ i ];

				// Descriptor is how we bind our buffers and images
				Descriptor descriptor = g_checkerboardShadowPipeline.GetFreeDescriptor();
				descriptor.BindBuffer( uniforms, camOffset, camSize, 0 );									// bind the camera matrices
				descriptor.BindBuffer( uniforms, renderModel.uboByteOffset, renderModel.uboByteSize, 1 );	// bind the model matrices
				descriptor.BindBuffer( uniforms, shadowCamOffset, shadowCamSize, 2 );						// bind the shadow camera matrices
				descriptor.BindImage( VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL, g_shadowFrameBuffer.m_imageDepth.m_vkImageView, Samplers::m_samplerStandard, 0 );
				descriptor.BindDescriptor( device, cmdBuffer, &g_checkerboardShadowPipeline );
				renderModel.model->DrawIndexed( cmdBuffer );

				// Debug drawing
				if (isDebug) {
					DrawDebugAxes(cmdBuffer, device, renderModel, uniforms);
				}
			}
		}

		g_offscreenFrameBuffer.EndRenderPass( device, cmdBufferIndex );

		g_offscreenFrameBuffer.m_imageColor.TransitionLayout( cmdBuffer, VK_IMAGE_LAYOUT_GENERAL );		
	}
}
