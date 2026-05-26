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

    if (filepath == "Shaders/convexhull_vert.wgsl")
    {
        return R"wgsl(//==============//
struct Uniform
{
    viewMatrix: mat4x4<f32>,
    projectionMatrix: mat4x4<f32>,
};

//==============//
struct Mat3x3
{
    col0: vec4<f32>,
    col1: vec4<f32>,
    col2: vec4<f32>,
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
};

struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

//==============//
@vertex
fn vs(input: VertexInput, @builtin(instance_index) instanceIndex: u32) -> VertexOutput
{
    var output: VertexOutput;
    let instance = instanceBuffer[instanceIndex];

    let worldPosition = instance.modelMatrix * vec4<f32>(input.position, 1.0);
    output.position = uniforms.projectionMatrix * uniforms.viewMatrix * worldPosition;

    let highlight = mix(instance.color.rgb, vec3<f32>(1.0, 0.95, 0.25), 0.7);
    output.color = vec4<f32>(highlight, 1.0);
    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/convexhull_frag.wgsl")
    {
        return R"wgsl(struct FragmentInput
{
    @location(0) color: vec4<f32>,
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

    if (filepath == "Shaders/background_vert.wgsl")
    {
        return R"wgsl(struct VertexOutput
{
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs(@builtin(vertex_index) vertexIndex: u32) -> VertexOutput
{
    let positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -3.0),
        vec2<f32>(3.0, 1.0),
        vec2<f32>(-1.0, 1.0)
    );

    var output: VertexOutput;
    let position = positions[vertexIndex];
    output.position = vec4<f32>(position, 0.0, 1.0);
    output.uv = position * 0.5 + vec2<f32>(0.5);
    return output;
}
)wgsl";
    }

    if (filepath == "Shaders/background_frag.wgsl")
    {
        return R"wgsl(struct BackgroundUniform
  {
      resolution: vec2<f32>,
      time: f32,
      tanHalfFovY: f32,
      cameraRight: vec4<f32>,
      cameraUp: vec4<f32>,
      cameraForward: vec4<f32>,
  };

@group(0) @binding(0)
var<uniform> background: BackgroundUniform;

fn skyRay(uv: vec2<f32>) -> vec3<f32>
  {
      let resolution = max(background.resolution, vec2<f32>(1.0));
      let aspect = resolution.x / resolution.y;
      let ndc = uv * 2.0 - vec2<f32>(1.0);
      let viewRay = normalize(vec3<f32>(
          ndc.x * aspect * background.tanHalfFovY,
          ndc.y * background.tanHalfFovY,
          1.0
      ));
  
      return normalize(
          background.cameraRight.xyz * viewRay.x +
          background.cameraUp.xyz * viewRay.y +
          background.cameraForward.xyz * viewRay.z
      );
  }

@fragment
fn fs(input: VertexOutput) -> @location(0) vec4<f32>
{
    let time = background.time * 0.34;
    let ray = skyRay(input.uv);
    let h = clamp(ray.y * 0.5 + 0.5, 0.0, 1.0);
    let horizon = 1.0 - smoothstep(0.0, 0.42, abs(ray.y + 0.035));
    let sky = smoothstep(-0.10, 0.42, ray.y);

    var q = vec2<f32>(
        ray.x * 1.45 + ray.z * 0.75,
        ray.z * 1.20 - ray.x * 0.55 + ray.y * 1.80
    );

    q.y += sin(q.x * 2.1 + time) * 0.23;
    q.x += sin(q.y * 3.0 - time * 0.9) * 0.17;
    q += vec2<f32>(
        sin((q.y + q.x) * 1.8 + time * 0.75),
        cos((q.x - q.y) * 1.7 - time * 0.65)
    ) * 0.08;

    let slowWave = sin(q.x * 3.2 + q.y * 1.8 + time * 1.3);
    let crossWave = sin((q.x - q.y) * 4.9 - time * 1.15);
    let fineWave = sin(length(q + vec2<f32>(0.28, -0.16)) * 8.8 - time * 2.0);
    let curlWave = sin((q.x * q.y) * 3.5 + time * 1.6);
    let weave = slowWave * 0.42 + crossWave * 0.30 + fineWave * 0.16 + curlWave * 0.12;
    let bands = smoothstep(-0.08, 0.78, weave * 0.5 + 0.5);
    let glow = pow(bands, 1.85);
    let drift = 0.5 + 0.5 * sin(q.x * 1.3 - q.y * 1.9 + time * 0.9);
    let hueShift = 0.5 + 0.5 * sin(time * 0.85 + ray.x * 1.6 + ray.z * 0.9);

    let nadir = vec3<f32>(0.008, 0.010, 0.014);
    let zenith = mix(vec3<f32>(0.012, 0.022, 0.044), vec3<f32>(0.026, 0.016, 0.044), hueShift);
    let horizonA = mix(vec3<f32>(0.024, 0.080, 0.086), vec3<f32>(0.080, 0.044, 0.090), hueShift);
    let horizonB = mix(vec3<f32>(0.070, 0.046, 0.030), vec3<f32>(0.030, 0.072, 0.088), drift);
    let colorPulse = 0.64 + 0.36 * sin(time * 1.4 + weave * 2.0);
    let cloud = mix(horizonA, horizonB, hueShift * 0.55) * glow * colorPulse;
    let base = mix(nadir, zenith, sky) + horizonA * horizon * 0.55;
    let color = base + cloud * (0.42 + 0.72 * horizon + 0.22 * h);

    return vec4<f32>(color, 1.0);
}
)wgsl";
    }

    return nullptr;
}

#endif // EMBEDDED_SHADERS_HPP
