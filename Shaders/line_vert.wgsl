//==============//
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