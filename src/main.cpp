#include "app/Application.h"
#include "configure/Config.h"

#include <iostream>
#include <exception>

int main()
{
    try
    {
        const RuntimeConfig config =
            RuntimeConfig::load(MECHANICS_CONFIG_PATH);

        std::cout
            << "Config loaded\n"
            << "Physics Hz: " << config.physics_hz << '\n'
            << "Fast Hz: " << config.fast_physics_hz << '\n'
            << "Restitution: " << config.restitution << '\n'
            << "CCD iterations: " << config.ccd_solver_iterations << '\n'
            << "Fast substeps: " << config.fastSubsteps() << '\n';

        Application app(config, MECHANICS_CONFIG_PATH);
        app.run();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}