#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "gltools.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define NUM_CELLS_X 80
#define NUM_CELLS_Y 80
#define N 1024 * 2 * 2
#define DT 0.004f

#define GSSB GL_SHADER_STORAGE_BUFFER
#define GLDC GL_DYNAMIC_COPY

typedef struct{
    float x;
    float y;
}Vector2;

void init_particles(Vector2 *positions, Vector2 *velocities, Vector2 *accelerations, float *masses, float *inv_masses, float *radii, int n, float max_m, float min_m, float max_r, float min_r, float spawn_radius, float center_x, float center_y) 
{
    for (int i = 0; i < n; i++) 
    {
        float rand_norm_m = (float)rand() / (float)RAND_MAX;
        float m = min_m + rand_norm_m * (max_m - min_m);
        masses[i] = m;
        inv_masses[i] = 1.0f / m;

        float rand_norm_r = (float)rand() / (float)RAND_MAX;
        float r = min_r + rand_norm_r * (max_r - min_r);
        radii[i] = r;

        velocities[i] = (Vector2){0.0f, 0.0f};
        accelerations[i] = (Vector2){0.0f, 0.0f};

        int max_attempts = 1000;
        int attempt = 0;
        int valid_position_found = 0;

        while (!valid_position_found && attempt < max_attempts) 
        {
            float rand_u = (float)rand() / (float)RAND_MAX;
            float rand_theta = ((float)rand() / (float)RAND_MAX) * 2.0f * 3.14159265f;

            float available_radius = spawn_radius - r;
            float current_radius = available_radius * sqrt(rand_u); 

            float px = center_x + current_radius * cos(rand_theta);
            float py = center_y + current_radius * sin(rand_theta);

            valid_position_found = 1;
            for (int j = 0; j < i; j++) 
            {
                float dx = px - positions[j].x;
                float dy = py - positions[j].y;
                float dist_sq = dx * dx + dy * dy;
                
                float min_dist = r + radii[j];

                if (dist_sq < (min_dist * min_dist)) 
                {
                    valid_position_found = 0; 
                    break; 
                }
            }

            if (valid_position_found) 
            {
                positions[i] = (Vector2){px, py};
            }
            
            attempt++;
        }
        if (!valid_position_found) 
        {
            printf("particle %d dropped in center", i);
            positions[i] = (Vector2){center_x, center_y};
        }
    }
}

int main()
{
    srand(time(NULL));

    //sim parameters
    float max_r = 8.0f;
    float min_r = 3.0f;

    float max_m = 2.0f;
    float min_m = 10.0f;

    float cell_size = 2.0f * 8;

    float G = -2.0f;
    float coefficient_of_restitution = 0.4f;
    float coefficient_of_friction = 0.0f;

    float v_max = cell_size/DT;
    float velocity_damping = 0.999f;

    int num_groups = N/256;
    int clear_value = -1;

    //dimensions related
    int total_cells = NUM_CELLS_X * NUM_CELLS_Y;

    int WIDTH = (int)(NUM_CELLS_X * cell_size);
    int HEIGHT = (int)(NUM_CELLS_Y * cell_size);

    //simulation data
    Vector2 *positions = malloc(N * sizeof(Vector2));
    Vector2 *velocities = malloc(N * sizeof(Vector2));
    Vector2 *accelerations = malloc(N * sizeof(Vector2));

    Vector2 *temp_positions = malloc(N * sizeof(Vector2));
    Vector2 *temp_velocities = malloc(N * sizeof(Vector2));

    int *cell_hashes = malloc(N * sizeof(int));
    int *cell_indices = malloc(N * sizeof(int));

    int *cell_starts = malloc(total_cells * sizeof(int));
    int *cell_ends = malloc(total_cells * sizeof(int));

    memset(cell_starts, -1, total_cells * sizeof(int));
    memset(cell_ends, -1, total_cells * sizeof(int));

    float *masses = malloc(N * sizeof(float));
    float *inv_masses = malloc(N * sizeof(float));
    float *radii = malloc(N * sizeof(float));

    //unit quad 
    float quad_vertices[] = {
        -0.5f, -0.5f, 
        0.5f, -0.5f, 
        -0.5f,  0.5f, 
        0.5f,  0.5f
    };

    float spawn_radius = (WIDTH / 2.0f) * 0.8f; 
    float center_x = WIDTH / 2.0f;
    float center_y = HEIGHT / 2.0f;

    init_particles(positions, velocities, accelerations, masses, inv_masses, radii, N, max_m, min_m, max_r, min_r, spawn_radius, center_x, center_y);

     //gl stuff
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "hahah", NULL, NULL);
    glfwMakeContextCurrent(window);
    glewInit();

    //read compute shaders
    char *verlet1_src = readFile("verlet_update1.comp");
    char *gravity_src = readFile("forces.comp");
    char *verlet2_src = readFile("verlet_update2.comp");
    char *cell_hashing_src = readFile("cell_hashing.comp");
    char *bitonic_sort_src = readFile("parallel_bitonic.comp");
    char *boundaries_src = readFile("start_end.comp");
    char *collision_resolution_src = readFile("collisions.comp");
    char *iteration_update_src = readFile("iteration_update.comp");

    //read vert and frag shaders
    char *vsrc = readFile("quad.vert");
    char *fsrc = readFile("quad.frag");

    //create compute shaders
    GLuint verlet1_program = createComputeShader(verlet1_src);
    GLuint gravity_program = createComputeShader(gravity_src);
    GLuint verlet2_program = createComputeShader(verlet2_src);
    GLuint cell_hashing_program = createComputeShader(cell_hashing_src);
    GLuint bitonic_sort_program = createComputeShader(bitonic_sort_src);
    GLuint boundaries_program = createComputeShader(boundaries_src);
    GLuint collision_resolution_program = createComputeShader(collision_resolution_src);
    GLuint iteration_update_program = createComputeShader(iteration_update_src);

    //vertex and fragment shader creation
    GLuint render_program = createProgram(vsrc, fsrc);

    //ssbo declaration 
    GLuint positions_ssbo, velocities_ssbo, accelerations_ssbo, 
    cell_hashes_ssbo, cell_indices_ssbo, 
    cell_starts_ssbo, cell_ends_ssbo, 
    temp_positions_ssbo, temp_velocities_ssbo,
    masses_ssbo, inv_masses_ssbo, radii_ssbo;

    //vao and vbo declaration
    GLuint quad_vao, quad_vbo;

    //ssbo creatiion, binding
    //positions
    glGenBuffers(1, &positions_ssbo);
    glBindBuffer(GSSB, positions_ssbo);
    glBufferData(GSSB, N * sizeof(Vector2), positions, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 0, positions_ssbo);

    //velocities
    glGenBuffers(1, &velocities_ssbo);
    glBindBuffer(GSSB, velocities_ssbo);
    glBufferData(GSSB, N * sizeof(Vector2), velocities, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 1, velocities_ssbo);

    //accelerations
    glGenBuffers(1, &accelerations_ssbo);
    glBindBuffer(GSSB, accelerations_ssbo);
    glBufferData(GSSB, N * sizeof(Vector2), accelerations, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 2, accelerations_ssbo);

    //cell hashes
    glGenBuffers(1, &cell_hashes_ssbo);
    glBindBuffer(GSSB, cell_hashes_ssbo);
    glBufferData(GSSB, N * sizeof(int), cell_hashes, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 6, cell_hashes_ssbo);

    //cell indices
    glGenBuffers(1, &cell_indices_ssbo);
    glBindBuffer(GSSB, cell_indices_ssbo);
    glBufferData(GSSB, N * sizeof(int), cell_indices, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 7, cell_indices_ssbo);

    //cell starts
    glGenBuffers(1, &cell_starts_ssbo);
    glBindBuffer(GSSB, cell_starts_ssbo);
    glBufferData(GSSB, total_cells * sizeof(int), cell_starts, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 8, cell_starts_ssbo);

    //cell ends
    glGenBuffers(1, &cell_ends_ssbo);
    glBindBuffer(GSSB, cell_ends_ssbo);
    glBufferData(GSSB, total_cells * sizeof(int), cell_ends, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 9, cell_ends_ssbo);

    //masses
    glGenBuffers(1, &masses_ssbo);
    glBindBuffer(GSSB, masses_ssbo);
    glBufferData(GSSB, N * sizeof(float), masses, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 3, masses_ssbo);  

    //inverse masses
    glGenBuffers(1, &inv_masses_ssbo);
    glBindBuffer(GSSB, inv_masses_ssbo);
    glBufferData(GSSB, N * sizeof(float), inv_masses, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 4, inv_masses_ssbo); 

    //radii
    glGenBuffers(1, &radii_ssbo);
    glBindBuffer(GSSB, radii_ssbo);
    glBufferData(GSSB, N * sizeof(float), radii, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 5, radii_ssbo); 

    //temp positions 
    glGenBuffers(1, &temp_positions_ssbo);
    glBindBuffer(GSSB, temp_positions_ssbo);
    glBufferData(GSSB, N * sizeof(Vector2), temp_positions, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 10, temp_positions_ssbo);

    //temp velocities
    glGenBuffers(1, &temp_velocities_ssbo);
    glBindBuffer(GSSB, temp_velocities_ssbo);
    glBufferData(GSSB, N * sizeof(Vector2), temp_velocities, GL_DYNAMIC_COPY);
    glBindBufferBase(GSSB, 11, temp_velocities_ssbo);


    //vao and vbo creation, binding
    glGenVertexArrays(1, &quad_vao);
    glGenBuffers(1, &quad_vbo);

    glBindVertexArray(quad_vao);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    //get locations
    //cell hashing shader locations
    GLint hashing_cell_size_loc = glGetUniformLocation(cell_hashing_program, "cell_size");
    GLint hashing_cells_loc = glGetUniformLocation(cell_hashing_program, "cells");
    GLint hashing_num_loc = glGetUniformLocation(cell_hashing_program, "num_particles");

    //sorting shader locations
    GLint sort_k_loc = glGetUniformLocation(bitonic_sort_program, "k");
    GLint sort_j_loc = glGetUniformLocation(bitonic_sort_program, "j");

    //boundary shader locations
    GLint boundaries_num_loc = glGetUniformLocation(boundaries_program, "num_particles");

    //vertex and fragment shader locations
    GLint draw_width_loc = glGetUniformLocation(render_program, "screen_width");
    GLint draw_height_loc = glGetUniformLocation(render_program, "screen_height");
    GLint draw_radius_loc = glGetUniformLocation(render_program, "particle_radius");
    GLint draw_target_density_loc = glGetUniformLocation(render_program, "target_density");

    //verlet step 1 locations
    GLint verlet1_dt_loc = glGetUniformLocation(verlet1_program, "dt");
    GLint verlet1_num_particles_loc = glGetUniformLocation(verlet1_program, "num_particles");

    //gravity locations
    GLint gravity_num_particles_loc = glGetUniformLocation(gravity_program, "num_particles");
    GLint gravity_G_loc = glGetUniformLocation(gravity_program, "G");

    //verlet step 2 locations
    GLint verlet2_dt_loc = glGetUniformLocation(verlet2_program, "dt");
    GLint verlet2_num_particles_loc = glGetUniformLocation(verlet2_program, "num_particles");
    GLint verlet2_vmax_loc = glGetUniformLocation(verlet2_program, "vmax");
    GLint verlet2_velocity_damping_loc = glGetUniformLocation(verlet2_program, "velocity_damping");  

    //collision shader locations
    GLint collision_num_particles_loc = glGetUniformLocation(collision_resolution_program, "num_particles");
    GLint collision_cells_loc = glGetUniformLocation(collision_resolution_program, "cells");
    GLint collision_cell_size_loc = glGetUniformLocation(collision_resolution_program, "cell_size");
    GLint collision_coefficient_of_restitution_loc = glGetUniformLocation(collision_resolution_program, "coefficient_of_restitution");
    GLint collision_coefficient_of_friction_loc = glGetUniformLocation(collision_resolution_program, "coefficient_of_friction");

    //iteration update shader locations
    GLint iter_num_particles_loc = glGetUniformLocation(iteration_update_program, "num_particles");

    //send constants to gpu
    //send cell size, (num_cells_x, num_cells_y), num_particles to hashing shader
    glUseProgram(cell_hashing_program);

    glUniform1f(hashing_cell_size_loc, cell_size);
    glUniform2i(hashing_cells_loc, NUM_CELLS_X, NUM_CELLS_Y);
    glUniform1i(hashing_num_loc, N);

    //send num_particles to boundaries shader
    glUseProgram(boundaries_program);

    glUniform1i(boundaries_num_loc, N);

    //verlet step 1
    glUseProgram(verlet1_program);
    
    glUniform1f(verlet1_dt_loc, DT);
    glUniform1i(verlet1_num_particles_loc, N);

    //graviaty shader
    glUseProgram(gravity_program);
    
    glUniform1i(gravity_num_particles_loc, N);
    glUniform1f(gravity_G_loc, G);

    //verlet step 2
    glUseProgram(verlet2_program);
    
    glUniform1f(verlet2_dt_loc, DT);
    glUniform1i(verlet2_num_particles_loc, N);
    glUniform1f(verlet2_vmax_loc, v_max);
    glUniform1f(verlet2_velocity_damping_loc, velocity_damping);

    //collision shader
    glUseProgram(collision_resolution_program);

    glUniform1i(collision_num_particles_loc, N);
    glUniform2i(collision_cells_loc, NUM_CELLS_X, NUM_CELLS_Y);
    glUniform1f(collision_cell_size_loc, cell_size);
    glUniform1f(collision_coefficient_of_restitution_loc, coefficient_of_restitution);
    glUniform1f(collision_coefficient_of_friction_loc, coefficient_of_friction);

    //iteration update
    glUseProgram(iteration_update_program);

    glUniform1i(iter_num_particles_loc, N);

    //send data to render program
    glUseProgram(render_program);

    glUniform1f(draw_width_loc, (float)(WIDTH));
    glUniform1f(draw_height_loc, (float)(HEIGHT));

    const double sim_step = 1.0 / 60.0;  
    const int substeps_per_step = 8;    
    const int iterations_per_substep = 4;
    double last_time = glfwGetTime();
    double accumulator = 0.0;

    while (!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        accumulator += now - last_time;
        last_time = now;

        if (accumulator > 0.25) accumulator = 0.25;
        while (accumulator >= sim_step)
        {
            for (int substeps = 0; substeps < substeps_per_step; substeps++)
            {
                //dispatch verlet step 1
                glUseProgram(verlet1_program);
                glDispatchCompute(num_groups, 1, 1);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                //dispatch gravity 
                glUseProgram(gravity_program);
                glDispatchCompute(num_groups, 1, 1);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                //dispatch verlet step 2
                glUseProgram(verlet2_program);
                glDispatchCompute(num_groups, 1, 1);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                //hashing
                glUseProgram(cell_hashing_program);
                glDispatchCompute(num_groups, 1, 1);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                //sorting
                glUseProgram(bitonic_sort_program);
                for (int k = 2; k <= N; k *= 2)
                {
                    for (int j = k/2; j > 0; j /= 2)
                    {
                        glUniform1i(sort_k_loc, k);
                        glUniform1i(sort_j_loc, j);

                        glDispatchCompute(num_groups, 1, 1);

                        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                    }
                }

                //clear cell starts and ends to -1
                glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

                glClearNamedBufferData(cell_starts_ssbo, GL_R32I, GL_RED_INTEGER, GL_INT, &clear_value);
                glClearNamedBufferData(cell_ends_ssbo, GL_R32I, GL_RED_INTEGER, GL_INT, &clear_value);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                //dispatch boundaries shader
                glUseProgram(boundaries_program);
                glDispatchCompute(num_groups, 1, 1);

                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                for (int iterations = 0; iterations < iterations_per_substep; iterations++)
                {
                    //collisions
                    glUseProgram(collision_resolution_program);
                    glDispatchCompute(num_groups, 1, 1);

                    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

                    //iteration update
                    glUseProgram(iteration_update_program);
                    glDispatchCompute(num_groups, 1, 1);

                    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                }
            }
            accumulator -= sim_step;
        }
        //drawing
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(render_program);

        glBindVertexArray(quad_vao);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, N);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwTerminate();

    return 0;
}