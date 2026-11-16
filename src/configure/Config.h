#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct RuntimeConfig
{
    int physics_hz = 60;
    int fast_physics_hz = 240;

    float restitution = 0.8f;
    float projectile_speed = 2000.0f;
    float gravity_y = 980.0f;

    int solver_iterations = 12;
    int ccd_solver_iterations = 48;
    int max_ccd_events = 64;

    float penetration_slop = 0.01f;
    float position_correction = 0.8f;
    float restitution_threshold = 60.0f;

    float contact_distance = 0.02f;
    float closing_speed_epsilon = 0.01f;

    float wall_thickness = 10.0f;

    int stack_count = 5;
    int system_regular_count = 20;
    int system_fast_count = 2;
    int system_obstacle_count = 5;

    float system_regular_half_size = 10.0f;
    float system_fast_half_size = 4.0f;

    double max_frame_time = 0.25;

    double fixedDt() const
    {
        return 1.0 / physics_hz;
    }

    int fastSubsteps() const
    {
        return fast_physics_hz / physics_hz;
    }

    void validate() const;

    static RuntimeConfig load(const std::string& path);
};
#endif