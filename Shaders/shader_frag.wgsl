//==============//
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