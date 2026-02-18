//==============//
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