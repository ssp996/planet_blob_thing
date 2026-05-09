#version 430 core

in vec2 local_pos;
in float parameter;

out vec4 FragColor;

void main()
{
    float dist = length(local_pos);

    if (dist > 0.5) {
        discard;
    }
    vec3 resting_color = vec3(0.0, 0.4, 0.8); 
    vec3 high_pressure_color = vec3(0.9, 0.0, 0.05);

    float color_blend_factor = clamp(parameter * 0.001, 0.0, 1.0);

    vec3 fluid_color = mix(resting_color, high_pressure_color, color_blend_factor);

    float alpha = smoothstep(0.5, 0.3, dist);
    FragColor = vec4(fluid_color, alpha);
}