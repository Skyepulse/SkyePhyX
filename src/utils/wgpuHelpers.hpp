#ifndef WGPU_HELPERS_HPP
#define WGPU_HELPERS_HPP

#include <webgpu/webgpu_cpp.h>
#include <dawn/webgpu_cpp_print.h>
#include <webgpu/webgpu_glfw.h>
#include <filesystem>

int wgpuCreateInstance(wgpu::Instance& instance, const std::vector<wgpu::InstanceFeatureName>& requiredFeatures);
int wgpuRequestAdapter(const wgpu::Instance& instance, wgpu::Adapter& adapter, wgpu::RequestAdapterOptions const* options);
int wgpuCreateDevice(const wgpu::Instance instance, const wgpu::Adapter& adapter, wgpu::Device& device, wgpu::DeviceDescriptor const* descriptor);

int getShaderCodeFromFile(const std::string& filepath, std::string& outShaderCode);
std::filesystem::path getExecutableDirectory();

constexpr size_t AlignUp(size_t v, size_t a)
{
    return (v + a - 1) & ~(a - 1);
}

#endif // WGPU_HELPERS_HPP