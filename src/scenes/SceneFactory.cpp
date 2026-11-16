#include "SceneFactory.h"
#include "../physics/PhysicsWorld.h"
#include "../configure/Config.h"


void SceneFactory::build(PhysicsWorld& world, SceneType scene, const RuntimeConfig& config)
{
    world.clear();
    switch (scene)
    {
    case SceneType::Wall:
        world.setGravity({ 0.0f, 0.0f });
        makeWall(world, config);
        break;

    case SceneType::MultipleBounces:
        world.setGravity({ 0.0f, 0.0f });
        makeMultipleBounces(world, config);
        break;

    case SceneType::Stack:
        world.setGravity({ 0.0f, config.gravity_y });
        makeStack(world, config);
        break;

    case SceneType::System:
        world.setGravity({ 0.0f, config.gravity_y });
        makeSystem(world, config);
        break;
    }
}

void SceneFactory::makeWall(PhysicsWorld& world, const RuntimeConfig& config)
{
    Body wall;
    wall.position = { 640.0f, 360.0f };
    wall.half_size = {
        config.wall_thickness * 0.5f,
        150.0f
    };

    world.addStatic(wall);

    Body projectile;
    projectile.position = { 620.0f, 360.0f };
    projectile.half_size = { 4.0f, 4.0f };
    projectile.velocity = { config.projectile_speed, 0.0f };
    projectile.inverse_mass = 1.0f;

    world.addFastDynamic(projectile);
}

void SceneFactory::makeMultipleBounces(PhysicsWorld& world, const RuntimeConfig& config)
{
    Body leftWall;
    leftWall.position = { 610.0f, 360.0f };
    leftWall.half_size = {
        config.wall_thickness * 0.5f,
        150.0f
    };

    world.addStatic(leftWall);

    makeWall(world, config);
}

void SceneFactory::makeStack(PhysicsWorld& world, const RuntimeConfig& config)
{
    Body floor;
    floor.position = { 640.0f, 650.0f };
    floor.half_size = { 600.0f, 20.0f };

    world.addStatic(floor);

    for (int i = 0; i < config.stack_count; ++i)
    {
        Body block;

        block.position = {
            640.0f,
            550.0f - static_cast<float>(i) * 65.0f
        };

        block.half_size = { 25.0f, 25.0f };
        block.inverse_mass = 1.0f;

        world.addRegularDynamic(block);
    }
}

void SceneFactory::makeSystem(PhysicsWorld& world, const RuntimeConfig& config)
{
    Body floor;
    floor.position = { 640.0f, 650.0f };
    floor.half_size = { 600.0f, 20.0f };

    world.addStatic(floor);

    const float obstacleHalfSize =
        config.system_regular_half_size * 2.0f;

    for (int i = 0; i < config.system_obstacle_count; ++i)
    {
        Body obstacle;

        obstacle.position = {
            280.0f + static_cast<float>(i) * 160.0f,
            500.0f
        };

        obstacle.half_size = {
            obstacleHalfSize,
            obstacleHalfSize
        };

        world.addStatic(obstacle);
    }

    for (int i = 0; i < config.system_regular_count; ++i)
    {
        const int column = i % 5;
        const int row = i / 5;

        Body body;

        body.position = {
            280.0f + static_cast<float>(column) * 160.0f,
            200.0f + static_cast<float>(row) * 60.0f
        };

        body.half_size = {
            config.system_regular_half_size,
            config.system_regular_half_size 
        };

        body.inverse_mass = 1.0f;

        world.addRegularDynamic(body);
    }

    for (int i = 0; i < config.system_fast_count; ++i)
    {
        Body projectile;

        projectile.position = {
            100.0f,
            220.0f + static_cast<float>(i) * 120.0f
        };

        projectile.half_size = {
            config.system_fast_half_size,
            config.system_fast_half_size
        };

        projectile.velocity = {
            config.projectile_speed,
            0.0f
        };

        projectile.inverse_mass = 4.0f;

        world.addFastDynamic(projectile);
    }
}