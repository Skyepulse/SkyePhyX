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
struct RingAccumulator 
{
    float samples[10]{};
    int count = 0;
    int head = 0;
    float sum = 0.0f;

    void push(float v) 
    {
        if (count < 10) 
        {
            samples[head] = v;
            sum += v;
            head = (head + 1) % 10;
            count++;
        } 
        else 
        {
            sum -= samples[head];
            samples[head] = v;
            sum += v;
            head = (head + 1) % 10;
        }
    }
    float average() const { return count > 0 ? sum / count : 0.0f; }
};

//================================//
struct RenderTimings
{
    RingAccumulator acquireSwapchain;
    RingAccumulator updateInstances;
    RingAccumulator updateLines;
    RingAccumulator updateDebug;
    RingAccumulator render;
    RingAccumulator present;
    RingAccumulator totalFrame;
};

//================================//
class RenderEngine
{
public:
    RenderEngine(WgpuBundle* wgpuBundle, Solver* solver): solver(solver), wgpuBundle(wgpuBundle)
    {
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
    void UpdateInstanceBuffer(std::vector<Mesh*>& meshes);
    void UpdateLineBuffer(const std::vector<GPULineData>& lineData);
    void UpdateDebugPointBuffer(const std::vector<GPUDebugPointData>& debugPointData);
    void SetSolverStepTime(float time) { this->solverStepTimeMs = time; }

    //================================//
    Camera* GetCamera() { return this->camera.get(); }

    bool debug = true;
    RenderTimings renderTimings;

private:
    WgpuBundle* wgpuBundle;
    Solver* solver;
    std::unique_ptr<Camera> camera;

    bool resizeFlag = true;

    wgpu::QuerySet gpuTimingQuerySet;
    wgpu::Buffer gpuTimingResolveBuffer;
    wgpu::Buffer gpuTimingReadbackBuffers[2];
    bool gpuTimingMapInFlight = false;
    int currentTimingWriteBuffer = 0;

    float gpuFrameTimeDrawMs = 0.0f;
    RingAccumulator gpuFrameDrawAccumulator;

    float solverStepTimeMs = 0.0f;
    TimingCtx timingCtx{this, 0};

    //=============== ATLAS =================//
    std::vector<GPUVertex> allVertices;
    std::vector<uint32_t> allIndices;
    wgpu::Buffer atlasVertexBuffer;
    wgpu::Buffer atlasIndexBuffer;

    //============== WGPU OBJECTS (Normal Pipeline) ==================//
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

    //============== WGPU OBJECTS (Lines Pipeline) ==================//
    wgpu::ShaderModule lineShaderModule;
    wgpu::PipelineLayout linePipelineLayout;
    wgpu::RenderPipeline lineRenderPipeline;
    wgpu::BindGroupLayout lineBindGroupLayout;
    wgpu::BindGroup lineBindGroup;
    wgpu::Buffer lineVertexBuffer;
    uint32_t maxLines = 0;
    uint32_t numLines = 0;

    //============== WGPU OBJECTS (Debug Pipeline) ==================//
    wgpu::ShaderModule debugShaderModule;
    wgpu::PipelineLayout debugPipelineLayout;
    wgpu::RenderPipeline debugRenderPipeline;
    wgpu::BindGroupLayout debugBindGroupLayout;
    wgpu::BindGroup debugBindGroup;
    wgpu::Buffer sphereTopologyVertexBuffer;
    wgpu::Buffer sphereTopologyIndexBuffer;
    uint32_t sphereTopologyVertexCount;
    wgpu::Buffer pointsInfoStorageBuffer;
    uint32_t maxDebugPoints = 0;
    uint32_t numDebugPoints = 0;

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