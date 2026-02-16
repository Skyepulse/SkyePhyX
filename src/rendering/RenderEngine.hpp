#ifndef RENDERENGINE_HPP
#define RENDERENGINE_HPP

// ImGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_wgpu.h"

#include "../utils/wgpuBundle.hpp"
#include "../utils/wgpuHelpers.hpp"
#include "../helpers/camera.hpp"

class RenderEngine;

//================================//
struct TimingCtx 
{
    RenderEngine* engine;
    int bufferIndex;
};

struct RenderInfo
{
    uint32_t width;
    uint32_t height;
    double time;
    bool resizeNeeded;
};

//================================//
class RenderEngine
{
public:
    RenderEngine(WgpuBundle* wgpuBundle)
    {
        this->wgpuBundle = wgpuBundle;
        WindowFormat windowFormat = wgpuBundle->GetWindowFormat();

        this->camera = std::make_unique<Camera>(
            static_cast<float>(windowFormat.width) / static_cast<float>(windowFormat.height),
            45.0f * (3.14159265f / 180.0f),
            0.1f,
            100.0f
        );

    }

    ~RenderEngine()
    {
        // ImGUI Cleanup
        ImGui_ImplWGPU_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    //================================//
    void Initialize();
    void Render(void* userData);

    //================================//
    Camera* GetCamera() { return this->camera.get(); }

private:
    WgpuBundle* wgpuBundle;
    std::unique_ptr<Camera> camera;

    bool resizeFlag = false;

    wgpu::QuerySet gpuTimingQuerySet;
    wgpu::Buffer gpuTimingResolveBuffer;
    wgpu::Buffer gpuTimingReadbackBuffers[2];
    bool gpuTimingMapInFlight = false;
    int currentTimingWriteBuffer = 0;

    float gpuFrameTimeDrawMs = 0.0f;
    std::vector<float> gpuFrameDrawAccumulator;
    float cpuFrameTimeMs = 0.0f;
    std::vector<float> cpuFrameAccumulator;


    //================================//
    void InitImGui();
    void RenderImGui(wgpu::RenderPassEncoder& pass);
    void InitializeGPUTimingQueries();
    void ReadTimingQueries();
};

#endif // RENDERENGINE_HPP