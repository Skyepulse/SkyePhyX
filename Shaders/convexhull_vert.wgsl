//==============//
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
