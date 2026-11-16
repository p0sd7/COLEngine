#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <vector>
#include "Body.h"
#include "PhysicsSettings.h"

class PhysicsWorld {
public:
	explicit PhysicsWorld(PhysicsSettings settings);

	void clear();
	void setGravity(Vec2 gravity);
	void addStatic(const Body& body);
	void addRegularDynamic(const Body& body);
	void addFastDynamic(const Body& body);

	void step(float dt, SimulationMode mode);

	int lastCcdEvents() const
	{
		return last_ccd_events_;
	}

	bool ccdLimitReached() const
	{
		return ccd_limit_reached_;
	}

	int lastFastSubsteps() const
	{
		return last_fast_substeps_;
	}

	float lastCcdDiscardedTime() const
	{
		return last_ccd_discarded_time_;
	}

	const std::vector<Body>& getStaticBodies() const;
	const std::vector<Body>& getRegularDynamicBodies() const;
	const std::vector<Body>& getFastDynamicBodies() const;
private:
	int last_fast_substeps_ = 0;
	int last_ccd_events_ = 0;
	bool ccd_limit_reached_ = false;
	float last_ccd_discarded_time_ = 0.0f;
	PhysicsSettings settings_;
	std::vector<Body> static_bodies_;
	std::vector<Body> regular_dynamic_bodies_;
	std::vector<Body> fast_dynamic_bodies_;
	std::vector<Body*> collectBodies();
	void stepMultiRate(float dt);
	void applyGravity(float dt);
	void advanceBodies(float dt);
	void solveOverlaps(const std::vector<Body*>& bodies);
	void stepDiscrete(float dt);
	void stepCCD(float dt);
	void solveCcdContacts(const std::vector<Body*>& bodies);

	unsigned int next_body_id_ = 1;
};

#endif // !PHYSICSWORLD_H
