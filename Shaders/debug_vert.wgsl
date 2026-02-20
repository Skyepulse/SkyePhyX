//==============//
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