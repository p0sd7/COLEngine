#ifndef PHYSICSTATS_H
#define PHYSICSTATS_H

#include <cstdint>

struct PhysicStats
{
    double elapsed = 0.0;

    std::uint64_t frames = 0;
    std::uint64_t steps = 0;
    std::uint64_t fast_substeps = 0;

    std::uint64_t ccd_events = 0;
    std::uint64_t ccd_limit_hits = 0;

    double physics_ms = 0.0;
    double ccd_discarded_seconds = 0.0;

    void reset()
    {
        *this = PhysicStats{};
    }
};

struct PhysicStatsView
{
    double ms_per_step = 0.0;
    double ms_per_frame = 0.0;
    double fast_steps_per_frame = 0.0;

    std::uint64_t ccd_events = 0;
    std::uint64_t ccd_limit_hits = 0;

    double ccd_discarded_ms = 0.0;
};

#endif // !PHYSICSTATS_H