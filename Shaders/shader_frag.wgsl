//==============//
@fragment
fn fs(input: VertexOutput) -> @location(0) vec4<f32>
{
    return input.color;
}