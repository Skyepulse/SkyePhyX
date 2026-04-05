#ifndef WGPU_BUNDLE_HPP
#define WGPU_BUNDLE_HPP

#include <string>
#include <webgpu/webgpu_cpp.h>
#ifndef __EMSCRIPTEN__
#include <dawn/webgpu_cpp_print.h>
#endif
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
    wgpu::TextureFormat GetSwapchainFormat() const { return this->swapchainFormat; }
    wgpu::Limits& GetLimits() { return this->limits; }
    GLFWwindow* GetGLFWWindow() { return this->window; }
    wgpu::RenderPassColorAttachment& GetColorAttachment(wgpu::TextureView& view)
    {
        tempColorAttachment.view = view;
        tempColorAttachment.loadOp = wgpu::LoadOp::Clear;
        tempColorAttachment.storeOp = wgpu::StoreOp::Store;
        tempColorAttachment.clearValue = { 0.0f, 0.0f, 0.0f, 1.0f };
        return tempColorAttachment;
    }
    
    wgpu::TextureFormat GetPreferedPresentationFormat() const
    {
        return this->swapchainFormat;
    }

    WindowFormat GetWindowFormat()
    {
        WindowFormat format{ nullptr, this->currentWidth, this->currentHeight, this->resizeFlag };
        if (this->resizeFlag)
            this->resizeFlag = false;

        return format;
    }

    void SafeCreateBuffer(const wgpu::BufferDescriptor* descriptor, wgpu::Buffer& outBuffer);
    bool EnsureSurfaceConfigured();
    void RequestSurfaceReconfigure() { this->surfaceNeedsReconfigure = true; }
    bool IsSurfaceRenderable() const { return this->surface && this->currentWidth > 0 && this->currentHeight > 0; }

    bool SupportsTimestampQuery()
    {
        return this->supportsTimestampQuery;
    }

private:
    void ComputeLimits();
    
    void InitializeInstance();
    void InitializeGraphics();
    static std::string StringViewToString(wgpu::StringView value);
    wgpu::TextureFormat ChooseSwapchainFormat(const wgpu::SurfaceCapabilities& capabilities) const;

    bool ConfigureSurface();
    void Resize(int newWidth, int newHeight);

    // WebGPU objects
    wgpu::Instance instance;
    wgpu::Adapter adapter;
    wgpu::Device device;

    // Surface specifics
    wgpu::Surface surface;
    wgpu::TextureFormat swapchainFormat = wgpu::TextureFormat::Undefined;

    // Window specifics (most is dirty marking for proper acquisition or reconfiguration)
    GLFWwindow* window;
    int currentWidth;
    int currentHeight;
    bool resizeFlag = false;
    bool surfaceConfigured = false;
    bool surfaceNeedsReconfigure = false;

    wgpu::Limits limits;

    bool supportsTimestampQuery = false;
    wgpu::RenderPassColorAttachment tempColorAttachment;

};

#endif // WGPU_BUNDLE_HPP
