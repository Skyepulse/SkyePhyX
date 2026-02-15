#ifndef WGPU_BUNDLE_HPP
#define WGPU_BUNDLE_HPP

#include <webgpu/webgpu_cpp.h>
#include <dawn/webgpu_cpp_print.h>
#include <webgpu/webgpu_glfw.h>

//================================//
struct WindowFormat
{
    GLFWwindow* window;
    int width;
    int height;
    bool resizeNeeded;
};

//================================//
class WgpuBundle
{
public:
    WgpuBundle(WindowFormat windowFormat);
    ~WgpuBundle();
    
    wgpu::Instance& GetInstance() { return this->instance; }
    wgpu::Adapter& GetAdapter() { return this->adapter; }
    wgpu::Device& GetDevice() { return this->device; }
    wgpu::Surface& GetSurface() { return this->surface; }
    wgpu::TextureFormat& GetSwapchainFormat() { return this->swapchainFormat; }
    wgpu::Limits& GetLimits() { return this->limits; }
    GLFWwindow* GetGLFWWindow() { return this->window; }

    WindowFormat GetWindowFormat()
    {
        WindowFormat format{ nullptr, this->currentWidth, this->currentHeight, this->resizeFlag };
        if (this->resizeFlag)
            this->resizeFlag = false;

        return format;
    }

    void SafeCreateBuffer(const wgpu::BufferDescriptor* descriptor, wgpu::Buffer& outBuffer);

    bool SupportsTimestampQuery()
    {
        return this->supportsTimestampQuery;
    }

private:
    void ComputeLimits();
    
    void InitializeInstance();
    void InitializeGraphics();

    void ConfigureSurface();
    void Resize(int newWidth, int newHeight);

    // WebGPU objects
    wgpu::Instance instance;
    wgpu::Adapter adapter;
    wgpu::Device device;

    // Surface specifics
    wgpu::Surface surface;
    wgpu::TextureFormat swapchainFormat;

    // Window specifics
    GLFWwindow* window;
    int currentWidth;
    int currentHeight;
    bool resizeFlag = false;

    wgpu::Limits limits;

    bool supportsTimestampQuery = false;
};

#endif // WGPU_BUNDLE_HPP