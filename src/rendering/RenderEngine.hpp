#ifndef RENDERENGINE_HPP
#define RENDERENGINE_HPP

// ImGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_wgpu.h"

#include "../utils/wgpuBundle.hpp"
#include "../utils/wgpuHelpers.hpp"
#include "../helpers/camera.hpp"
#include "../helpers/geometry.hpp"
#include "../helpers/geometryGenerator.hpp"

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
struct GPUVertex
{
    float position[3];
    float normal[3];
    float UV[2];
};
static_assert(sizeof(GPUVertex) == 32, "GPUVertex must be tightly packed to 32 bytes for correct buffer layout");

struct GPUInstanceData
{
    float modelMatrix[16];  // 64
    GPUMat3x3 normalMatrix; // 48
    float color[4];         // 16
};
static_assert(sizeof(GPUInstanceData) == 128, "GPUInstanceData must be tightly packed to 128 bytes for correct buffer layout");

//================================//
struct MeshSlot
{
    uint32_t vertexOffset;
    uint32_t indexOffset;
    uint32_t indexCount;
};

struct ModelBatch
{
    ModelType modelType;
    MeshSlot slot;
    std::vector<GPUInstanceData> cpuInstances;
    uint32_t firstInstanceOffset;
    uint32_t instanceCount;
};

//================================//
struct Uniform
{
    float ViewMatrix[16];
    float ProjectionMatrix[16];
};
static_assert(sizeof(Uniform) == 128, "Uniform must be tightly packed to 128 bytes for correct buffer layout");

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
    void AcquireSwapchainTexture();
    void Render(void* userData);
    void UpdateInstanceBuffer(Mesh* bodies);
    void SetSolverStepTime(float time) { this->solverStepTimeMs = time; }

    //================================//
    Camera* GetCamera() { return this->camera.get(); }

private:
    WgpuBundle* wgpuBundle;
    std::unique_ptr<Camera> camera;

    bool resizeFlag = true;

    wgpu::QuerySet gpuTimingQuerySet;
    wgpu::Buffer gpuTimingResolveBuffer;
    wgpu::Buffer gpuTimingReadbackBuffers[2];
    bool gpuTimingMapInFlight = false;
    int currentTimingWriteBuffer = 0;

    float gpuFrameTimeDrawMs = 0.0f;
    std::vector<float> gpuFrameDrawAccumulator;
    float cpuFrameTimeMs = 0.0f;
    std::vector<float> cpuFrameAccumulator;
    float solverStepTimeMs = 0.0f;

    std::chrono::steady_clock::time_point cpuStartTime;

    //=============== ATLAS =================//
    std::vector<GPUVertex> allVertices;
    std::vector<uint32_t> allIndices;
    wgpu::Buffer atlasVertexBuffer;
    wgpu::Buffer atlasIndexBuffer;

    //============== WGPU OBJECTS ==================//
    wgpu::ShaderModule shaderModule;

    wgpu::PipelineLayout pipelineLayout;
    wgpu::RenderPipeline renderPipeline;

    wgpu::BindGroupLayout instanceBindGroupLayout;
    wgpu::BindGroup instanceBindGroup;

    wgpu::Buffer instanceDataBuffer;
    uint32_t maxInstances = 0;
    std::vector<GPUInstanceData> packedInstances;

    std::unordered_map<ModelType, ModelBatch> modelBatches;

    wgpu::TextureView depthTextureView;
    wgpu::Texture depthTexture;

    wgpu::Buffer uniformBuffer;
    wgpu::SurfaceTexture currentTexture;

    //================================//
    void InitImGui();
    void RenderImGui(wgpu::RenderPassEncoder& pass);
    void InitializeGPUTimingQueries();
    void ReadTimingQueries();

    void BuildGeometryAtlas();
    void BuildPipeline();
    void BuildBuffers();
    void BuildResizeDependentResources(float newWidth, float newHeight);
};

#endif // RENDERENGINE_HPP