//================================//
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