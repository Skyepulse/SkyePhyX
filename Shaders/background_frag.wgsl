struct BackgroundUniform
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
