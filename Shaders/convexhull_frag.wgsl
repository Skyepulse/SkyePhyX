struct FragmentInput
{
    @location(0) color: vec4<f32>,
};

//================================//
@fragment
fn fs(input: FragmentInput) -> @location(0) vec4<f32>
{
    return input.color;
}
