#ifndef SCENEFACTORY_H
#define SCENEFACTORY_H

class PhysicsWorld;
struct RuntimeConfig;

enum class SceneType {
	Wall,
	MultipleBounces,
	Stack,
	System
};

class SceneFactory
{
public:
    static void build(
        PhysicsWorld& world,
        SceneType scene,
        const RuntimeConfig& config
    );

private:
    static void makeWall(
        PhysicsWorld& world,
        const RuntimeConfig& config
    );

    static void makeMultipleBounces(
        PhysicsWorld& world,
        const RuntimeConfig& config
    );

    static void makeStack(
        PhysicsWorld& world,
        const RuntimeConfig& config
    );

    static void makeSystem(
        PhysicsWorld& world,
        const RuntimeConfig& config
    );
};
#endif // !SCENEFACTORY_H
