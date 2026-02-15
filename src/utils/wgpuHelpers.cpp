#include "wgpuHelpers.hpp"

#include <string>
#include <fstream>
#include <iostream>

#ifdef _WIN32
    #include <windows.h>
#elif defined(__EMSCRIPTEN__)
    #include <emscripten.h>
    #include <emscripten/fetch.h>
#elif defined(__linux__) || defined(__APPLE__)
    #include <limits.h>
    #include <unistd.h>
#endif

std::filesystem::path getExecutableDirectory() 
{
    #ifdef _WIN32
        // Windows
        wchar_t exePath[MAX_PATH];
        GetModuleFileNameW(NULL, exePath, MAX_PATH);
        return std::filesystem::path(exePath).parent_path();

    #elif defined(__EMSCRIPTEN__)
        // Emscripten (Web)
        // Note: Emscripten doesn't have a traditional filesystem,
        // so we return a virtual path or handle assets differently.
        // For WebGPU/WASM, you might want to use EM_ASYNC_JS or preload assets.
        return std::filesystem::path("/"); // Placeholder; adjust as needed

    #elif defined(__linux__) || defined(__APPLE__)
        // Linux/macOS
        char exePath[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
        if (len == -1) {
            throw std::runtime_error("Failed to get executable path");
        }
        exePath[len] = '\0';
        return std::filesystem::path(exePath).parent_path();

    #else
        #error "Unsupported platform"
    #endif
}

//================================//
int wgpuCreateInstance(wgpu::Instance& instance, const std::vector<wgpu::InstanceFeatureName>& requiredFeatures)
{
    wgpu::InstanceDescriptor descriptor{};
    descriptor.requiredFeatureCount = static_cast<uint32_t>(requiredFeatures.size());
    descriptor.requiredFeatures = requiredFeatures.data();

    instance = wgpu::CreateInstance(&descriptor);

    if (!instance)
        return -1;
    return 0;
}

//================================//
int wgpuRequestAdapter(const wgpu::Instance& instance, wgpu::Adapter& adapter, wgpu::RequestAdapterOptions const* options)
{
    bool requestCompleted = false;
    
    wgpu::Future f1 = instance.RequestAdapter(
        nullptr,
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::RequestAdapterStatus status, wgpu::Adapter a, wgpu::StringView message, wgpu::Adapter* userdata)
        {
            if (status == wgpu::RequestAdapterStatus::Success)
                *userdata = std::move(a); 
        },
        &adapter
    );
    instance.WaitAny(f1, UINT64_MAX); // Timeout to max

    if (!adapter)
        return -1;
    return 0;
}

//================================//
int wgpuCreateDevice(const wgpu::Instance instance, const wgpu::Adapter& adapter, wgpu::Device& device, wgpu::DeviceDescriptor const* descriptor)
{
    bool requestCompleted = false;

    wgpu::Future f2 = adapter.RequestDevice(
        descriptor,
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::RequestDeviceStatus status, wgpu::Device d, wgpu::StringView message, wgpu::Device* userdata)
        {
            if (status == wgpu::RequestDeviceStatus::Success)
                *userdata = std::move(d); 
        },
        &device
    );
    instance.WaitAny(f2, UINT64_MAX); // Timeout to max

    if (!device)
        return -1;
    return 0;
}

//================================//
int getShaderCodeFromFile(const std::string& filepath, std::string& outShaderCode) 
{   
#ifdef __EMSCRIPTEN__
    FILE* file = fopen(filepath.c_str(), "rb");
    if (!file) 
    {
        std::cout << "[wgpuHelpers] Failed to open shader file: " << filepath << std::endl;
        return -1;
    }

    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);

    outShaderCode.resize(size);
    if (size > 0) {
        fread(outShaderCode.data(), 1, size, file);
    }
    fclose(file);
#else
    std::filesystem::path shaderPath = getExecutableDirectory() / filepath;
    std::ifstream file(shaderPath, std::ios::ate | std::ios::binary);
    if (!file) {
        std::cout << "[wgpuHelpers] Failed to open shader file: " << shaderPath << std::endl;
        return -1;
    }

    // Get file size
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read all content
    outShaderCode.resize(size);
    if (size > 0) {
        file.read(outShaderCode.data(), size);
    }
#endif

    return 0;
}