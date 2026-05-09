#version 430 core

layout(location = 0) in vec2 quad_vertex;

layout(std430, binding = 0) readonly buffer PositionBuffer 
{
    vec2 positions[];
};

layout(std430, binding = 1) readonly buffer VelocityBuffer 
{
    vec2 velocities[];
};

layout(std430, binding = 5) readonly buffer RadiusBuffer 
{
    float radii[];
};

uniform float screen_width;
uniform float screen_height;

out vec2 local_pos;
out float parameter;

void main()
{
    vec2 particle_center = positions[gl_InstanceID];
    vec2 scaled_vertex = quad_vertex * 2.0 * radii[gl_InstanceID];
    vec2 world_pos = scaled_vertex + particle_center;

    float ndc_x = (world_pos.x / screen_width) * 2.0 - 1.0;
    float ndc_y = (world_pos.y / screen_height) * 2.0 - 1.0;

    gl_Position = vec4(ndc_x, ndc_y, 0.0, 1.0);

    local_pos = quad_vertex;
    vec2 vel_i = velocities[gl_InstanceID];
    parameter = dot(vel_i, vel_i);
}