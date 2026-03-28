#ifndef EMBEDDED_SHADERS_HPP
#define EMBEDDED_SHADERS_HPP

#include <string>

inline const char* GetEmbeddedShaderSource(const std::string& filepath)
{
    if (filepath == "Shaders/shader_vert.wgsl")
    {
        return R"wgsl(//==============//
struct Mat3x3
{
    col0: vec4<f32>,
    col1: vec4<f32>,
    col2: vec4<f32>,
};

//==============//
struct Uniform
{
    viewMatrix: mat4x4<f32>,
    projectionMatrix: mat4x4<f32>,
};

//==============//
struct GPUInstance
{
    modelMatrix: mat4x4<f32>,
    normalMatrix: Mat3x3,
    color: vec4<f32>,
};

@group(0) @binding(0) 
var<uniform> uniforms: Uniform;
@group(0) @binding(1)
var<storage, read> instanceBuffer: array<GPUInstance>;

struct VertexInput
{
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) UV: vec2<f32>,
};

struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) UV: vec2<f32>,
    @location(2) color: vec4<f32>,
    @location(3) worldPos: vec3<f32>,
};

//==============//
fn mulMat3x3Vec3(m: Mat3x3, v: vec3<f32>) -> vec3<f32>
{
    return m.col0.xyz * v.x + m.col1.xyz * v.y + m.col2.xyz * v.z;
}

//==============//
@vertex
fn vs(input: VertexInput, @builtin(instance_index) instanceIndex: u32) -> VertexOutput
{
    var output: VertexOutput;
    let instance = instanceBuffer[instanceIndex];
    
    let worldPosition = instance.modelMatrix * vec4<f32>(input.position, 1.0);
    output.position = uniforms.projectionMatrix * uniforms.viewMatrix * worldPosition;
    
    let worldNormal = mulMat3x3Vec3(instance.normalMatrix, input.normal);
    output.normal = normalize(worldNormal);
    
    output.UV = input.UV;
    output.color = instance.color;

    output.worldPos = worldPosition.xyz;
    
    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/shader_frag.wgsl")
    {
        return R"wgsl(//==============//
@fragment
fn fs(input: VertexOutput) -> @location(0) vec4<f32>
{
    let lightDir = normalize(vec3<f32>(-0.5, 1.0, 0.5));
    let normal = normalize(input.normal);

    let diffuse = max(dot(normal, lightDir), 0.0);
    let ambient = 0.15;

    let lighting = diffuse + ambient;
    let color = input.color.rgb * lighting;

    return vec4<f32>(color, input.color.a);
}
)wgsl";
    }

    if (filepath == "Shaders/line_vert.wgsl")
    {
        return R"wgsl(//==============//
struct Uniform
{
    viewMatrix: mat4x4<f32>,
    projectionMatrix: mat4x4<f32>,
};

@group(0) @binding(0) 
var<uniform> uniforms: Uniform;

struct VertexInput
{
    @location(0) start: vec3<f32>,
    @location(1) end: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(2) color: vec4<f32>,
};

//==============//
@vertex
fn vs(input: VertexInput, @builtin(vertex_index) vertexIndex: u32) -> VertexOutput
{
    var output: VertexOutput;
    
    var worldPos: vec3<f32>;
    if (vertexIndex % 2u == 0u) 
    {
        worldPos = input.start;
    } 
    else 
    {
        worldPos = input.end;
    }
    
    let viewPos = uniforms.viewMatrix * vec4<f32>(worldPos, 1.0);
    output.position = uniforms.projectionMatrix * viewPos;
    output.color = input.color;

    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/line_frag.wgsl")
    {
        return R"wgsl(struct FragmentInput
{
    @location(2) color: vec4<f32>,
};

@fragment
fn fs(input: FragmentInput) -> @location(0) vec4<f32>
{
    return input.color;
}
)wgsl";
    }

    if (filepath == "Shaders/debug_vert.wgsl")
    {
        return R"wgsl(//==============//
struct Uniform
{
    viewMatrix: mat4x4<f32>,
    projectionMatrix: mat4x4<f32>,
};

//==============//
struct GPUDebugPointData
{
    position: vec3<f32>,
    color: vec4<f32>,
};

@group(0) @binding(0) 
var<uniform> uniforms: Uniform;
@group(0) @binding(1)
var<storage, read> debugPointBuffer: array<GPUDebugPointData>;

struct VertexInput
{
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) UV: vec2<f32>,
};

struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) UV: vec2<f32>,
    @location(2) color: vec4<f32>,
    @location(3) worldPos: vec3<f32>,
};

//==============//
@vertex
fn vs(input: VertexInput, @builtin(instance_index) instanceIndex: u32) -> VertexOutput
{
    var output: VertexOutput;
    let instance = debugPointBuffer[instanceIndex];
    
    let scale = 0.05f;
    let worldPosition = vec4<f32>(input.position * scale + instance.position, 1.0);
    output.position = uniforms.projectionMatrix * uniforms.viewMatrix * worldPosition;
    
    output.normal = input.normal;
    
    output.UV = input.UV;
    output.color = instance.color;

    output.worldPos = worldPosition.xyz;
    
    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/debug_frag.wgsl")
    {
        return R"wgsl(//==============//
@fragment
fn fs(input: VertexOutput) -> @location(0) vec4<f32>
{
    let lightDir = normalize(vec3<f32>(-0.5, 1.0, 0.5));
    let normal = normalize(input.normal);

    let diffuse = max(dot(normal, lightDir), 0.0);
    let ambient = 0.15;

    let lighting = diffuse + ambient;
    let color = input.color.rgb * lighting;

    return vec4<f32>(color, input.color.a);
}
)wgsl";
    }

    if (filepath == "Shaders/softbody_vert.wgsl")
    {
        return R"wgsl(//================================//
struct Uniform
{
    viewMatrix: mat4x4<f32>,
    projectionMatrix: mat4x4<f32>,
};

//================================//
@group(0) @binding(0)
var<uniform> uniforms: Uniform;

//================================//
struct VertexInput
{
    @location(0) pos: vec3<f32>,
    @location(1) norm: vec3<f32>,
    @location(2) color: vec4<f32>,
};

//================================//
struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(0) viewNorm: vec3<f32>,
    @location(1) color: vec4<f32>,
};

//================================//
@vertex
fn vs(input: VertexInput) -> VertexOutput
{
    var output: VertexOutput;
    let viewPos = uniforms.viewMatrix * vec4<f32>(input.pos, 1.0);
    output.position = uniforms.projectionMatrix * viewPos;
    let normalMat = mat3x3<f32>(
        uniforms.viewMatrix[0].xyz,
        uniforms.viewMatrix[1].xyz,
        uniforms.viewMatrix[2].xyz
    );
    output.viewNorm = normalMat * input.norm;
    output.color = input.color;
    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/softbody_frag.wgsl")
    {
        return R"wgsl(//================================//
struct FragmentInput
{
    @location(0) viewNorm: vec3<f32>,
    @location(1) color: vec4<f32>,
};

//================================//
@fragment
fn fs(input: FragmentInput) -> @location(0) vec4<f32>
{
    let lightDir = normalize(vec3<f32>(0.5, 1.0, 0.7));
    let n = normalize(input.viewNorm);
    let diff = abs(dot(n, lightDir));
    let lit = 0.25 + 0.75 * diff;
    return vec4<f32>(input.color.rgb * lit, input.color.a);
}
)wgsl";
    }

    return nullptr;
}

#endif // EMBEDDED_SHADERS_HPP
