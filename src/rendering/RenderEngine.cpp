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


    ImGui_ImplWGPU_Init(&init_info);
}

//================================//
void RenderEngine::RenderImGui(wgpu::RenderPassEncoder& pass)
{
    ImGui_ImplWGPU_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Render();
    ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass.Get());
}

//================================//
void RenderEngine::Initialize()
{
    this->InitializeGPUTimingQueries();
    this->InitImGui();
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
        const float aspectRatio = static_cast<float>(windowFormat.width) / static_cast<float>(windowFormat.height);
        this->camera->SetAspectRatio(aspectRatio);
    }

    // INIT
    wgpu::SurfaceTexture currentTexture;
    wgpu::Surface surface = this->wgpuBundle->GetSurface();
    surface.GetCurrentTexture(&currentTexture);

    auto status = currentTexture.status;
    if (status != wgpu::SurfaceGetCurrentTextureStatus::SuccessOptimal &&
        status != wgpu::SurfaceGetCurrentTextureStatus::SuccessSuboptimal)
    {
        std::cout << "[ERROR][RenderEngine] Surface lost/outdated, need reconfigure.\n";
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

    // Draw pass
    {
        wgpu::RenderPassDescriptor renderPassDesc{};
        renderPassDesc.colorAttachmentCount = 1;
        renderPassDesc.colorAttachments = &this->wgpuBundle->GetColorAttachment(swapchainView);
        if (this->wgpuBundle->SupportsTimestampQuery())
            renderPassDesc.timestampWrites = &drawTimestamps;

        wgpu::RenderPassEncoder pass = encoder.BeginRenderPass(&renderPassDesc);
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