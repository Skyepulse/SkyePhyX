#include "RenderEngine.hpp"
#include <numeric>
#include <iostream>

//================================//
void RenderEngine::InitImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    GLFWwindow* window = this->wgpuBundle->GetGLFWWindow();
    ImGui_ImplGlfw_InitForOther(window, true);

    wgpu::Device device = this->wgpuBundle->GetDevice();
    WindowFormat windowFormat = this->wgpuBundle->GetWindowFormat();

    ImGui_ImplWGPU_InitInfo init_info = {};
    init_info.Device = device.Get();
    init_info.NumFramesInFlight = 3;
    init_info.RenderTargetFormat = WGPUTextureFormat_BGRA8Unorm;
    init_info.DepthStencilFormat = WGPUTextureFormat_Depth24Plus;

    ImGui_ImplWGPU_Init(&init_info);
}

//================================//
void RenderEngine::RenderImGui(wgpu::RenderPassEncoder& pass)
{
    ImGui_ImplWGPU_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::Render();
    ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass.Get());
}

//================================//
void RenderEngine::Initialize()
{
    this->InitializeGPUTimingQueries();
    this->InitImGui();

    this->BuildPipeline();
    this->BuildGeometryAtlas();
    this->BuildBuffers();
}

//================================//
void RenderEngine::Render(void* userData)
{
    auto renderInfo = *static_cast<RenderInfo*>(userData);
    if (renderInfo.resizeNeeded)
        this->resizeFlag = true;

    if (this->resizeFlag)
    {
        WindowFormat windowFormat = this->wgpuBundle->GetWindowFormat();
        this->BuildResizeDependentResources(static_cast<float>(windowFormat.width), static_cast<float>(windowFormat.height));
    }

    // INIT
    wgpu::SurfaceTexture currentTexture;
    wgpu::Surface surface = this->wgpuBundle->GetSurface();
    surface.GetCurrentTexture(&currentTexture);

    auto status = currentTexture.status;
    if (status != wgpu::SurfaceGetCurrentTextureStatus::SuccessOptimal &&
        status != wgpu::SurfaceGetCurrentTextureStatus::SuccessSuboptimal)
    {
        return;
    }

    auto cpuFrameStart = std::chrono::high_resolution_clock::now();

    wgpu::TextureView swapchainView = currentTexture.texture.CreateView();
    wgpu::Queue queue = this->wgpuBundle->GetDevice().GetQueue();
    wgpu::CommandEncoder encoder = this->wgpuBundle->GetDevice().CreateCommandEncoder();

    wgpu::PassTimestampWrites drawTimestamps{};
    drawTimestamps.querySet = this->gpuTimingQuerySet;
    drawTimestamps.beginningOfPassWriteIndex = 0;
    drawTimestamps.endOfPassWriteIndex = 1;

    wgpu::RenderPassDepthStencilAttachment depthAttachment{};
    depthAttachment.view = this->depthTextureView;
    depthAttachment.depthLoadOp = wgpu::LoadOp::Clear;
    depthAttachment.depthStoreOp = wgpu::StoreOp::Store;
    depthAttachment.depthClearValue = 1.0f;

    Uniform frameUniforms{};
    Eigen::Matrix4f view, proj;
    this->camera->GetViewMatrix(view);
    this->camera->GetProjectionMatrix(proj);
    Eigen::Map<Eigen::Matrix4f>(frameUniforms.ViewMatrix) = view;
    Eigen::Map<Eigen::Matrix4f>(frameUniforms.ProjectionMatrix) = proj;
    this->wgpuBundle->GetDevice().GetQueue().WriteBuffer(this->uniformBuffer, 0, &frameUniforms, sizeof(Uniform));

    // Draw pass
    {
        wgpu::RenderPassDescriptor renderPassDesc{};
        renderPassDesc.colorAttachmentCount = 1;
        renderPassDesc.colorAttachments = &this->wgpuBundle->GetColorAttachment(swapchainView);
        renderPassDesc.depthStencilAttachment = &depthAttachment;
        if (this->wgpuBundle->SupportsTimestampQuery())
            renderPassDesc.timestampWrites = &drawTimestamps;

        wgpu::RenderPassEncoder pass = encoder.BeginRenderPass(&renderPassDesc);
        pass.SetPipeline(this->renderPipeline.Get());

        pass.SetBindGroup(0, this->instanceBindGroup.Get());
        pass.SetVertexBuffer(0, this->atlasVertexBuffer.Get());
        pass.SetIndexBuffer(this->atlasIndexBuffer.Get(), wgpu::IndexFormat::Uint32);

        for (const auto& [modelType, batch] : this->modelBatches)
        {
            if (batch.instanceCount == 0)
                continue;

            pass.DrawIndexed(
                batch.slot.indexCount, 
                batch.instanceCount, 
                batch.slot.indexOffset, 
                batch.slot.vertexOffset, 
                batch.firstInstanceOffset
            );
        }
        
        this->RenderImGui(pass);
        pass.End();
    }

    if (this->wgpuBundle->SupportsTimestampQuery())
    {
        // Resolve timestamps
        encoder.ResolveQuerySet(
            this->gpuTimingQuerySet,
            0,
            2,
            this->gpuTimingResolveBuffer,
            0
        );

        encoder.CopyBufferToBuffer(
            this->gpuTimingResolveBuffer,
            0,
            this->gpuTimingReadbackBuffers[currentTimingWriteBuffer],
            0,
            sizeof(uint64_t) * 2
        );
    }

    wgpu::CommandBuffer commandBuffer = encoder.Finish();
    this->wgpuBundle->GetDevice().GetQueue().Submit(1, &commandBuffer);

    auto cpuFrameEnd = std::chrono::high_resolution_clock::now();
    double cpuFrameMs = std::chrono::duration<double, std::milli>(cpuFrameEnd - cpuFrameStart).count();
    this->cpuFrameAccumulator.push_back(static_cast<float>(cpuFrameMs));
    if (this->cpuFrameAccumulator.size() > 10)
        this->cpuFrameAccumulator.erase(this->cpuFrameAccumulator.begin());
    this->cpuFrameTimeMs = std::accumulate(this->cpuFrameAccumulator.begin(), this->cpuFrameAccumulator.end(), 0.0f) / static_cast<float>(this->cpuFrameAccumulator.size());
}

//================================//
void RenderEngine::InitializeGPUTimingQueries()
{
    if (!this->wgpuBundle->SupportsTimestampQuery())
        return;

    wgpu::QuerySetDescriptor queryDesc{};
    queryDesc.type = wgpu::QueryType::Timestamp;
    queryDesc.count = 2;
    this->gpuTimingQuerySet = this->wgpuBundle->GetDevice().CreateQuerySet(&queryDesc);

    wgpu::BufferDescriptor resolveDesc{};
    resolveDesc.size = sizeof(uint64_t) * 2;
    resolveDesc.usage = wgpu::BufferUsage::QueryResolve | wgpu::BufferUsage::CopySrc;
    this->gpuTimingResolveBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&resolveDesc);

    wgpu::BufferDescriptor readbackDesc{};
    readbackDesc.size = sizeof(uint64_t) * 2;
    readbackDesc.usage = wgpu::BufferUsage::MapRead | wgpu::BufferUsage::CopyDst;
    for (int i = 0; i < 2; ++i)
    {
        this->gpuTimingReadbackBuffers[i] = this->wgpuBundle->GetDevice().CreateBuffer(&readbackDesc);
    }
}

//================================//
void RenderEngine::ReadTimingQueries()
{
    if(!this->wgpuBundle->SupportsTimestampQuery() || this->gpuTimingMapInFlight)
        return;

    int readBuffer = currentTimingWriteBuffer;
    currentTimingWriteBuffer = 1 - currentTimingWriteBuffer; // swap
    this->gpuTimingMapInFlight = true;
    
    TimingCtx* ctx = new TimingCtx{this, readBuffer};
    
    this->gpuTimingReadbackBuffers[readBuffer].MapAsync(
        wgpu::MapMode::Read,
        0,
        sizeof(uint64_t) * 2,
        wgpu::CallbackMode::AllowProcessEvents,
        [](wgpu::MapAsyncStatus status, wgpu::StringView message, TimingCtx* ctx) {
            if (status == wgpu::MapAsyncStatus::Success)
            {
                const uint64_t* timestamps = static_cast<const uint64_t*>(
                    ctx->engine->gpuTimingReadbackBuffers[ctx->bufferIndex].GetConstMappedRange()
                );
                
                float drawMs = (timestamps[1] - timestamps[0]) / 1'000'000.0f;

                ctx->engine->gpuFrameDrawAccumulator.push_back(drawMs);
                if (ctx->engine->gpuFrameDrawAccumulator.size() > 10)
                    ctx->engine->gpuFrameDrawAccumulator.erase(ctx->engine->gpuFrameDrawAccumulator.begin());
                ctx->engine->gpuFrameTimeDrawMs = std::accumulate(
                    ctx->engine->gpuFrameDrawAccumulator.begin(),
                    ctx->engine->gpuFrameDrawAccumulator.end(), 0.0f
                ) / ctx->engine->gpuFrameDrawAccumulator.size();
                
                ctx->engine->gpuTimingReadbackBuffers[ctx->bufferIndex].Unmap();
            }
            
            ctx->engine->gpuTimingMapInFlight = false;
            delete ctx;
        },
        ctx
    );
}

//================================//
void RenderEngine::BuildResizeDependentResources(float newWidth, float newHeight)
{
    const float aspectRatio = newWidth / newHeight;
    this->camera->SetAspectRatio(aspectRatio);

    // Recreate depth texture
    wgpu::TextureDescriptor depthDesc{};
    depthDesc.size.width = static_cast<uint32_t>(newWidth);
    depthDesc.size.height = static_cast<uint32_t>(newHeight);
    depthDesc.size.depthOrArrayLayers = 1;
    depthDesc.mipLevelCount = 1;
    depthDesc.sampleCount = 1;
    depthDesc.dimension = wgpu::TextureDimension::e2D;
    depthDesc.format = wgpu::TextureFormat::Depth24Plus;
    depthDesc.usage = wgpu::TextureUsage::RenderAttachment;

    this->depthTexture = this->wgpuBundle->GetDevice().CreateTexture(&depthDesc);
    this->depthTextureView = this->depthTexture.CreateView();
}

//================================//
void RenderEngine::BuildPipeline()
{
    // 1. SHADER
    std::string vertShaderCode;
    if (getShaderCodeFromFile("Shaders/shader_vert.wgsl", vertShaderCode) < 0)
    {
        throw std::runtime_error(
            "[ERROR][RenderEngine] Failed to load vertex shader code from path: " +
            (getExecutableDirectory() / "Shaders/shader_vert.wgsl").string()
        );
    }

    std::string fragShaderCode;
    if (getShaderCodeFromFile("Shaders/shader_frag.wgsl", fragShaderCode) < 0)
    {
        throw std::runtime_error(
            "[ERROR][RenderEngine] Failed to load fragment shader code from path: " +
            (getExecutableDirectory() / "Shaders/shader_frag.wgsl").string()
        );
    }

    std::string combinedShaderCode = vertShaderCode + "\n" + fragShaderCode;

    wgpu::ShaderSourceWGSL wgsl{};
    wgsl.code = combinedShaderCode.c_str();

    wgpu::ShaderModuleDescriptor shaderDesc{};
    shaderDesc.nextInChain = &wgsl;
    shaderDesc.label = "MainShaderModule";

    this->shaderModule = this->wgpuBundle->GetDevice().CreateShaderModule(&shaderDesc);
    if (!this->shaderModule)
    {
        throw std::runtime_error("[ERROR][RenderEngine] Failed to create shader module.");
    }

    // 2. Bind Group Layout
    wgpu::BindGroupLayoutEntry entries[2]{};
    entries[0].binding = 0;
    entries[0].visibility = wgpu::ShaderStage::Vertex;
    entries[0].buffer.type = wgpu::BufferBindingType::Uniform;

    entries[1].binding = 1;
    entries[1].visibility = wgpu::ShaderStage::Vertex;
    entries[1].buffer.type = wgpu::BufferBindingType::ReadOnlyStorage;

    wgpu::BindGroupLayoutDescriptor bindGroupLayoutDesc{};
    bindGroupLayoutDesc.label = "InstanceDataBindGroupLayout";
    bindGroupLayoutDesc.entryCount = 2;
    bindGroupLayoutDesc.entries = entries;

    this->instanceBindGroupLayout = this->wgpuBundle->GetDevice().CreateBindGroupLayout(&bindGroupLayoutDesc);

    // 3. Pipeline Layout
    wgpu::PipelineLayoutDescriptor pipelineLayoutDesc{};
    pipelineLayoutDesc.label = "MainPipelineLayout";
    pipelineLayoutDesc.bindGroupLayoutCount = 1;
    pipelineLayoutDesc.bindGroupLayouts = &this->instanceBindGroupLayout;

    this->pipelineLayout = this->wgpuBundle->GetDevice().CreatePipelineLayout(&pipelineLayoutDesc);

    // 4. Render Pipeline

    wgpu::VertexAttribute attributes[3]{};
    attributes[0].shaderLocation = 0;
    attributes[0].format = wgpu::VertexFormat::Float32x3; // position
    attributes[0].offset = offsetof(GPUVertex, position);

    attributes[1].shaderLocation = 1;
    attributes[1].format = wgpu::VertexFormat::Float32x3; // normal
    attributes[1].offset = offsetof(GPUVertex, normal);

    attributes[2].shaderLocation = 2;
    attributes[2].format = wgpu::VertexFormat::Float32x2; // UV
    attributes[2].offset = offsetof(GPUVertex, UV);

    wgpu::VertexBufferLayout vertexBufferLayout{}; // Geometry atlas vertex buffer layout
    vertexBufferLayout.arrayStride = sizeof(GPUVertex);
    vertexBufferLayout.attributeCount = 3;
    vertexBufferLayout.attributes = attributes;

    wgpu::VertexState vertexState{};
    vertexState.module = this->shaderModule;
    vertexState.entryPoint = "vs";
    vertexState.bufferCount = 1;
    vertexState.buffers = &vertexBufferLayout;

    wgpu::ColorTargetState colorTarget{};
    colorTarget.format = this->wgpuBundle->GetPreferedPresentationFormat();
    colorTarget.writeMask = wgpu::ColorWriteMask::All;

    wgpu::FragmentState fragmentState{};
    fragmentState.module = this->shaderModule;
    fragmentState.entryPoint = "fs";
    fragmentState.targetCount = 1;
    fragmentState.targets = &colorTarget;

    wgpu::PrimitiveState primitiveState{};
    primitiveState.topology = wgpu::PrimitiveTopology::TriangleList;
    primitiveState.cullMode = wgpu::CullMode::Back;

    wgpu::DepthStencilState depthStencilState{};
    depthStencilState.format = wgpu::TextureFormat::Depth24Plus;
    depthStencilState.depthWriteEnabled = true;
    depthStencilState.depthCompare = wgpu::CompareFunction::Less;

    wgpu::RenderPipelineDescriptor pipelineDesc{};
    pipelineDesc.label = "MainRenderPipeline";
    pipelineDesc.layout = this->pipelineLayout;
    pipelineDesc.vertex = vertexState;
    pipelineDesc.fragment = &fragmentState;
    pipelineDesc.primitive = primitiveState;
    pipelineDesc.depthStencil = &depthStencilState;

    this->renderPipeline = this->wgpuBundle->GetDevice().CreateRenderPipeline(&pipelineDesc);
}

//================================//
void RenderEngine::BuildGeometryAtlas()
{
    this->allVertices.clear();
    this->allIndices.clear();
    this->modelBatches.clear();

    struct Entry { ModelType type; MeshData data; };

    std::vector<Entry> entries = {
        { ModelType_Cube, GeometryGenerator::GenerateCube() },
        { ModelType_Sphere, GeometryGenerator::GenerateSphere(32, 16) },
        // { ModelType_Pyramid, GeometryGenerator::CreatePyramid() }
    };

    uint32_t totalVertices = 0;
    uint32_t totalIndices = 0;

    for (const Entry& entry : entries)
    {
        MeshSlot slot;

        slot.vertexOffset = totalVertices;
        slot.indexOffset = totalIndices;
        slot.indexCount = entry.data.indexCount();

        this->modelBatches[entry.type] = ModelBatch
        {
            entry.type,
            slot,
            {},
            0,
            0
        };

        for (const Vertex& v : entry.data.vertices)
        {
            GPUVertex gv{};
            Eigen::Map<Eigen::Vector3f>(gv.position) = v.position;
            Eigen::Map<Eigen::Vector3f>(gv.normal)   = v.normal;
            Eigen::Map<Eigen::Vector2f>(gv.UV)       = v.uv;
            this->allVertices.push_back(gv);
        }

        for (const uint32_t& i : entry.data.indices)
        {
            this->allIndices.push_back(i);
        }

        totalVertices += entry.data.vertexCount();
        totalIndices += entry.data.indexCount();
    }
}

//================================//
void RenderEngine::BuildBuffers()
{
    // Geometry Atlas Buffers
    wgpu::BufferDescriptor vertexBufferDesc{};
    vertexBufferDesc.label = "AtlasVertexBuffer";
    vertexBufferDesc.size = sizeof(GPUVertex) * this->allVertices.size();
    vertexBufferDesc.usage = wgpu::BufferUsage::Vertex | wgpu::BufferUsage::CopyDst;
    this->atlasVertexBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&vertexBufferDesc);
    this->wgpuBundle->GetDevice().GetQueue().WriteBuffer(
        this->atlasVertexBuffer,
        0,
        this->allVertices.data(),
        sizeof(GPUVertex) * this->allVertices.size()
    );

    wgpu::BufferDescriptor indexBufferDesc{};
    indexBufferDesc.label = "AtlasIndexBuffer";
    indexBufferDesc.size = sizeof(uint32_t) * this->allIndices.size();
    indexBufferDesc.usage = wgpu::BufferUsage::Index | wgpu::BufferUsage::CopyDst;
    this->atlasIndexBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&indexBufferDesc);
    this->wgpuBundle->GetDevice().GetQueue().WriteBuffer(
        this->atlasIndexBuffer,
        0,
        this->allIndices.data(),
        sizeof(uint32_t) * this->allIndices.size()
    );

    // Instances
    this->maxInstances = 4096;
    const uint32_t instanceDataBufferSize = sizeof(GPUInstanceData) * this->maxInstances;

    wgpu::BufferDescriptor instanceBufferDesc{};
    instanceBufferDesc.label = "InstanceDataBuffer";
    instanceBufferDesc.size = instanceDataBufferSize;
    instanceBufferDesc.usage = wgpu::BufferUsage::Storage | wgpu::BufferUsage::CopyDst;
    this->instanceDataBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&instanceBufferDesc);

    // Bind group for instance data
    Uniform InitUniform{};
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    Eigen::Map<Eigen::Matrix4f>(InitUniform.ViewMatrix) = identity;
    Eigen::Map<Eigen::Matrix4f>(InitUniform.ProjectionMatrix) = identity;

    wgpu::BufferDescriptor uniformBufferDesc{};
    uniformBufferDesc.label = "UniformBuffer";
    uniformBufferDesc.size = sizeof(Uniform);
    uniformBufferDesc.usage = wgpu::BufferUsage::Uniform | wgpu::BufferUsage::CopyDst;
    this->uniformBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&uniformBufferDesc);
    this->wgpuBundle->GetDevice().GetQueue().WriteBuffer(
        this->uniformBuffer,
        0,
        &InitUniform,
        sizeof(Uniform)
    );

    wgpu::BindGroupEntry bindGroupEntries[2]{};
    bindGroupEntries[0].binding = 0;
    bindGroupEntries[0].buffer = this->uniformBuffer;
    bindGroupEntries[0].offset = 0;
    bindGroupEntries[0].size = sizeof(Uniform);

    bindGroupEntries[1].binding = 1;
    bindGroupEntries[1].buffer = this->instanceDataBuffer;
    bindGroupEntries[1].offset = 0;
    bindGroupEntries[1].size = instanceDataBufferSize;

    wgpu::BindGroupDescriptor bindGroupDesc{};
    bindGroupDesc.label = "InstanceDataBindGroup";
    bindGroupDesc.layout = this->instanceBindGroupLayout;
    bindGroupDesc.entryCount = 2;
    bindGroupDesc.entries = bindGroupEntries;
    this->instanceBindGroup = this->wgpuBundle->GetDevice().CreateBindGroup(&bindGroupDesc);
}

//================================//
void RenderEngine::UpdateInstanceBuffer(Mesh* bodies)
{
    for (auto& [modelType, batch] : this->modelBatches)
    {
        batch.cpuInstances.clear();
        batch.instanceCount = 0;
        batch.firstInstanceOffset = 0;
    }

    Eigen::Matrix4f modelMatrixBuffer = Eigen::Matrix4f::Identity();
    Eigen::MatrixXf normalMatrixBuffer(3, 4);

    int meshCount = 0;
    for (const Mesh* body = bodies; body != nullptr; body = body->next)
    {
        auto it = this->modelBatches.find(body->modelType);
        if (it == this->modelBatches.end()) continue; // unknown model type??

        GPUInstanceData instanceData{};
        body->transform.GetModelMatrix(modelMatrixBuffer);
        body->transform.GetNormalMatrix(instanceData.normalMatrix);
        Eigen::Map<Eigen::Matrix4f>(instanceData.modelMatrix) = modelMatrixBuffer;
        Eigen::Vector3f color = body->color;
        instanceData.color[0] = color.x();
        instanceData.color[1] = color.y();
        instanceData.color[2] = color.z();
        instanceData.color[3] = 1.0f;

        it->second.cpuInstances.push_back(instanceData);
        meshCount++;
    }

    this->packedInstances.clear();
    for (auto& [modelType, batch] : this->modelBatches)
    {
        if (batch.cpuInstances.empty()) continue;

        batch.firstInstanceOffset = static_cast<uint32_t>(this->packedInstances.size());
        batch.instanceCount = static_cast<uint32_t>(batch.cpuInstances.size());
        this->packedInstances.insert(this->packedInstances.end(), batch.cpuInstances.begin(), batch.cpuInstances.end());
    }

    if (this->packedInstances.size() > this->maxInstances)
    {
        this->maxInstances = this->packedInstances.size() * 2;
        
        wgpu::BufferDescriptor instanceBufferDesc{};
        instanceBufferDesc.label = "InstanceDataBuffer";
        instanceBufferDesc.size = sizeof(GPUInstanceData) * this->maxInstances;
        instanceBufferDesc.usage = wgpu::BufferUsage::Storage | wgpu::BufferUsage::CopyDst;
        this->instanceDataBuffer = this->wgpuBundle->GetDevice().CreateBuffer(&instanceBufferDesc);

        wgpu::BindGroupEntry bindGroupEntries[2]{};

        bindGroupEntries[0].binding = 0;
        bindGroupEntries[0].buffer = this->uniformBuffer;
        bindGroupEntries[0].offset = 0;
        bindGroupEntries[0].size = sizeof(Uniform);

        bindGroupEntries[1].binding = 1;
        bindGroupEntries[1].buffer = this->instanceDataBuffer;
        bindGroupEntries[1].offset = 0;
        bindGroupEntries[1].size = sizeof(GPUInstanceData) * this->maxInstances;

        wgpu::BindGroupDescriptor bindGroupDesc{};
        bindGroupDesc.label = "InstanceDataBindGroup";
        bindGroupDesc.layout = this->instanceBindGroupLayout;
        bindGroupDesc.entryCount = 2;
        bindGroupDesc.entries = bindGroupEntries;

        this->instanceBindGroup = this->wgpuBundle->GetDevice().CreateBindGroup(&bindGroupDesc);
    }

    if (!this->packedInstances.empty())
    {
        this->wgpuBundle->GetDevice().GetQueue().WriteBuffer(
            this->instanceDataBuffer,
            0,
            this->packedInstances.data(),
            sizeof(GPUInstanceData) * this->packedInstances.size()
        );
    }
}