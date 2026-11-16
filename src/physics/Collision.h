#ifndef COLLISION_H
#define COLLISION_H

#include <optional>
#include "Body.h"
#include "PhysicsSettings.h"

namespace Collision {
	struct Contact {
		Vec2 normal;
		float penetration = 0.0f;
	};

	struct SweepHit
	{
		float time = 0.0f;
		Vec2 normal;
	};

	std::optional<SweepHit> sweepAabb(
		const Body& a,
		const Body& b,
		float maxTime
	);

	std::optional<Contact> findStaticContact(
		const Body& moving,
		const Body& obstacle
	);

	std::optional<Contact> findTouchingContact(
		const Body& a,
		const Body& b,
		float tolerance
	);

	std::optional<Contact> findContact(
		const Body& a,
		const Body& b
	);

	void resolveStaticContact(
		Body& moving,
		const Contact& contact,
		float restitution
	);

	void resolveContact(
		Body& a,
		Body& b,
		const Contact& contact,
		const PhysicsSettings& settings
	);
}

#endif // !COLLISION_H
