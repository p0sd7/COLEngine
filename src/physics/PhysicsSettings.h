#ifndef PHYSICSSETTINGS_H
#define PHYSICSSETTINGS_H

#include "Math.h"

enum class SimulationMode
{
    Discrete,
    CCD,
    MultiRate
};

struct PhysicsSettings
{
    Vec2 gravity;

    float restitution;

    int solver_iterations;

    float penetration_slop;
    float position_correction;
    float restitution_threshold;
    int max_ccd_events;
    int fast_substeps;
    int ccd_solver_iterations;
    float contact_distance;
    float closing_speed_epsilon;
};

#endif