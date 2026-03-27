//================================//
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
