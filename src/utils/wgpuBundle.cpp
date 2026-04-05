#include "wgpuBundle.hpp"
#include "wgpuHelpers.hpp"

#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

//================================//
std::string WgpuBundle::StringViewToString(wgpu::StringView value)
{
    return std::string(static_cast<std::string_view>(value));
}

//================================//
wgpu::TextureFormat WgpuBundle::ChooseSwapchainFormat(const wgpu::SurfaceCapabilities& capabilities) const
{
    constexpr wgpu::TextureFormat kPreferredFormats[] = {
        wgpu::TextureFormat::BGRA8Unorm,
        wgpu::TextureFormat::RGBA8Unorm,
        wgpu::TextureFormat::BGRA8UnormSrgb,
        wgpu::TextureFormat::RGBA8UnormSrgb,
    };

    for (wgpu::TextureFormat preferredFormat : kPreferredFormats)
    {
        for (size_t i = 0; i < capabilities.formatCount; ++i)
        {
            if (capabilities.formats[i] == preferredFormat)
                return preferredFormat;
        }
    }

    if (capabilities.formatCount == 0)
        return wgpu::TextureFormat::Undefined;

    return capabilities.formats[0];
}

//================================//
WgpuBundle::WgpuBundle(WindowFormat windowFormat) : window(windowFormat.window), currentWidth(windowFormat.width), currentHeight(windowFormat.height)
{
    this->InitializeInstance();

    if (this->window != nullptr)
    {
#ifdef __EMSCRIPTEN__
        wgpu::EmscriptenSurfaceSourceCanvasHTMLSelector canvasDesc{};
        canvasDesc.selector = "#canvas";

        wgpu::SurfaceDescriptor surfaceDesc{};
        surfaceDesc.nextInChain = &canvasDesc;
        this->surface = this->instance.CreateSurface(&surfaceDesc);
#else
        this->surface = wgpu::glfw::CreateSurfaceForWindow(this->instance, window);
#endif

        // Callback on window resize
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(
            window,
            [](GLFWwindow* wnd, int width, int height)
            {
                WgpuBundle* bundle = static_cast<WgpuBundle*>(glfwGetWindowUserPointer(wnd));
                bundle->Resize(width, height);
            }
        );
    }

    this->InitializeGraphics();
}

//================================//
WgpuBundle::~WgpuBundle()
{
    std::cout << "[wgpuBundle][Shutdown] Cleaning up WebGPU resources..." << std::endl;

    if (this->surface)
    {
        this->surface.Unconfigure();
        this->surface = nullptr;  // Release surface while device is still valid
    }

    // Wait for all GPU work to complete before destroying resources
    if (this->device)
    {
        wgpu::Queue queue = this->device.GetQueue();
        if (queue)
        {
            wgpu::Future workDoneFuture = queue.OnSubmittedWorkDone(
                wgpu::CallbackMode::WaitAnyOnly,
                [](wgpu::QueueWorkDoneStatus status, wgpu::StringView message) {}
            );
            this->instance.WaitAny(workDoneFuture, UINT64_MAX);
        }
        this->device.Destroy();
    }
}

//================================//
void WgpuBundle::InitializeInstance()
{
    // Instance required features
    const wgpu::InstanceFeatureName kTimeWaitAny = wgpu::InstanceFeatureName::TimedWaitAny;
    std::vector<wgpu::InstanceFeatureName> requiredFeatures = { kTimeWaitAny };

    if (wgpuCreateInstance(this->instance, requiredFeatures) < 0)
        throw std::runtime_error("Failed to create WebGPU instance.");

    // Adapter
    wgpu::RequestAdapterOptions options{};
    options.backendType = wgpu::BackendType::Undefined;
    if (wgpuRequestAdapter(this->instance, this->adapter, &options) < 0)
        throw std::runtime_error("Failed to request WebGPU adapter.");

    adapter.GetLimits(&this->limits);
    this->ComputeLimits();

    this->supportsTimestampQuery = this->adapter.HasFeature(wgpu::FeatureName::TimestampQuery);

    wgpu::DeviceDescriptor deviceDesc{};
    deviceDesc.requiredLimits = &this->limits;
    deviceDesc.SetUncapturedErrorCallback(
        [](const wgpu::Device&, wgpu::ErrorType errorType, wgpu::StringView message)
        {
            std::cout << "[wgpuDevice] Uncaptured error: " << StringViewToString(message) << std::endl;
        }
    );
    deviceDesc.SetDeviceLostCallback(
        wgpu::CallbackMode::WaitAnyOnly,
        [](const wgpu::Device& device, wgpu::DeviceLostReason reason, wgpu::StringView message)
        {
            std::cerr << "[wgpuDevice] Device lost! Reason: " << static_cast<int>(reason) 
                    << ", Message: " << StringViewToString(message) << std::endl;
        }
    );

    wgpu::FeatureName requiredDeviceFeatures[] = {
        wgpu::FeatureName::TimestampQuery,
    };

    if (this->supportsTimestampQuery)
    {
        std::cout << "[wgpuBundle][Init] Timestamp query supported, enabling feature.\n";
        deviceDesc.requiredFeatures = requiredDeviceFeatures;
        deviceDesc.requiredFeatureCount = 1;
    }
    else
    {
        std::cout << "[wgpuBundle][Init] Timestamp query not supported, GPU timing unavailable.\n";
        deviceDesc.requiredFeatureCount = 0;
        deviceDesc.requiredFeatures = nullptr;
    }
    if (wgpuCreateDevice(this->instance, this->adapter, this->device, &deviceDesc) < 0)
        throw std::runtime_error("Failed to create WebGPU device.");

    wgpu::AdapterInfo info;
    adapter.GetInfo(&info);
    std::cout << "[wgpuBundle][Init] Using adapter: " << StringViewToString(info.description) << std::endl;
    std::cout << "[wgpuBundle][Init] Using device: " << StringViewToString(info.device) << std::endl;
    std::cout << "[wgpuBundle][Init] Device limits: " << std::endl;
    std::cout << "  - Max buffer size: " << this->limits.maxBufferSize << " GB: " << static_cast<double>(this->limits.maxBufferSize) / (1024 * 1024 * 1024) << std::endl;
    std::cout << "  - Max storage buffer binding size: " << this->limits.maxStorageBufferBindingSize <<  " GB: " << static_cast<double>(this->limits.maxStorageBufferBindingSize) / (1024 * 1024 * 1024) << std::endl;
    std::cout << "[wgpuBundle][Init] Backend: " << static_cast<uint32_t>(info.backendType) << std::endl;
}

//================================//
void WgpuBundle::InitializeGraphics()
{
    this->ConfigureSurface();
}

//================================//
void WgpuBundle::ConfigureSurface()
{
    if (!this->surface)
        return;
        
    wgpu::SurfaceCapabilities capabilities;
    surface.GetCapabilities(adapter, &capabilities);
    swapchainFormat = ChooseSwapchainFormat(capabilities);
    if (swapchainFormat == wgpu::TextureFormat::Undefined)
        throw std::runtime_error("Surface does not expose a compatible presentation format.");

    wgpu::SurfaceConfiguration config{};
    config.device = device;
    config.format = swapchainFormat;
    config.width = currentWidth;
    config.height = currentHeight;
    config.usage = wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::CopySrc;

    surface.Configure(&config);
}

//================================//
void WgpuBundle::ComputeLimits()
{
    // None
}

//================================//
void WgpuBundle::Resize(int newWidth, int newHeight)
{
    if (newWidth <= 0 || newHeight <= 0)
        return;
        
    this->currentWidth = newWidth;
    this->currentHeight = newHeight;
    this->ConfigureSurface();
    this->resizeFlag = true;
}

//================================//
void WgpuBundle::SafeCreateBuffer(const wgpu::BufferDescriptor* descriptor, wgpu::Buffer& outBuffer)
{
    const uint64_t bufferRequestedSize = descriptor->size;
    if (bufferRequestedSize > this->limits.maxBufferSize)
    {
        std::cout << "[WgpuBundle] Requested buffer size (" << bufferRequestedSize
                    << ") exceeds device maxBufferSize limit (" << this->limits.maxBufferSize << ")." << std::endl;
        throw std::runtime_error("[WgpuBundle] Requested buffer size exceeds device limits.");
    }

    outBuffer = this->device.CreateBuffer(descriptor);
    if (!outBuffer)
    {
        std::cout << "[WgpuBundle] Failed to create buffer of size " << bufferRequestedSize << "." << std::endl;
        throw std::runtime_error("[WgpuBundle] Failed to create buffer.");
    }
}
