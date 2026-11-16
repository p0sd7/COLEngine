#include "Collision.h"

#include <cmath>
#include <algorithm>
#include <limits>

namespace Collision
{
	std::optional<Contact> findTouchingContact(
		const Body& a,
		const Body& b,
		float tolerance
	)
	{
		const Vec2 delta = a.position - b.position;

		const float overlapX =
			a.half_size.x + b.half_size.x - std::abs(delta.x);

		const float overlapY =
			a.half_size.y + b.half_size.y - std::abs(delta.y);

		if (overlapX < -tolerance || overlapY < -tolerance)
		{
			return std::nullopt;
		}

		Contact contact;

		if (overlapX < overlapY)
		{
			contact.normal = {
				delta.x >= 0.0f ? 1.0f : -1.0f,
				0.0f
			};

			contact.penetration = std::max(overlapX, 0.0f);
		}
		else
		{
			contact.normal = {
				0.0f,
				delta.y >= 0.0f ? 1.0f : -1.0f
			};

			contact.penetration = std::max(overlapY, 0.0f);
		}

		return contact;
	}	

	std::optional<SweepHit> sweepAabb(
		const Body& a,
		const Body& b,
		float maxTime
	)
	{
		if (maxTime < 0.0f)
		{
			return std::nullopt;
		}

		if (findContact(a, b))
		{
			return std::nullopt;
		}

		const Vec2 position = a.position - b.position;
		const Vec2 velocity = a.velocity - b.velocity;
		const Vec2 extent = a.half_size + b.half_size;

		const float infinity =
			std::numeric_limits<float>::infinity();

		float entryX = -infinity;
		float exitX = infinity;

		float entryY = -infinity;
		float exitY = infinity;

		const auto axisTimes = [](
			float position,
			float velocity,
			float extent,
			float& entry,
			float& exit
			) -> bool
			{
				if (velocity == 0.0f)
				{
					return std::abs(position) < extent;
				}

				const float t1 = (-extent - position) / velocity;
				const float t2 = (extent - position) / velocity;

				entry = std::min(t1, t2);
				exit = std::max(t1, t2);

				return true;
			};

		if (!axisTimes(
			position.x, velocity.x, extent.x, entryX, exitX))
		{
			return std::nullopt;
		}

		if (!axisTimes(
			position.y, velocity.y, extent.y, entryY, exitY))
		{
			return std::nullopt;
		}

		const float entryTime = std::max(entryX, entryY);
		const float exitTime = std::min(exitX, exitY);

		if (entryTime > exitTime ||
			exitTime < 0.0f ||
			entryTime < 0.0f ||
			entryTime > maxTime)
		{
			return std::nullopt;
		}

		Vec2 normal;

		if (entryX >= entryY)
		{
			normal = {
				velocity.x > 0.0f ? -1.0f : 1.0f,
				0.0f
			};
		}
		else
		{
			normal = {
				0.0f,
				velocity.y > 0.0f ? -1.0f : 1.0f
			};
		}

		if (dot(velocity, normal) >= 0.0f)
		{
			return std::nullopt;
		}

		return SweepHit{ entryTime, normal };
	}

	std::optional<Contact> findStaticContact(
		const Body& moving,
		const Body& obstacle
	)
	{
		const Vec2 delta = moving.position - obstacle.position;

		const float overlapX = moving.half_size.x + obstacle.half_size.x - std::abs(delta.x);
		const float overlapY = moving.half_size.y + obstacle.half_size.y - std::abs(delta.y);

		if (overlapX <= 0.0f || overlapY <= 0.0f) {
			return std::nullopt;
		}

		Contact contact;

		if (overlapX < overlapY) {
			contact.normal = {
				delta.x >= 0.0f ? 1.0f : -1.0f,
				0.0f
			};
			contact.penetration = overlapX;
		}
		else
		{
			contact.normal = {
				0.0f,
				delta.y >= 0.0f ? 1.0f : -1.0f
			};
			contact.penetration = overlapY;
		}

		return contact;
	}

	std::optional<Contact> findContact(
		const Body& a,
		const Body& b
	)
	{
		const Vec2 delta = a.position - b.position;
		const float overlapX = a.half_size.x + b.half_size.x - std::abs(delta.x);
		const float overlapY = a.half_size.y + b.half_size.y - std::abs(delta.y);

		if (overlapX <= 0.0f || overlapY <= 0.0f) {
			return std::nullopt;
		}

		Contact contact;

		if (overlapX < overlapY) {
			contact.normal = {
				delta.x >= 0.0f ? 1.0f : -1.0f,
				0.0f
			};
			contact.penetration = overlapX;
		}
		else
		{
			contact.normal = {
				0.0f,
				delta.y >= 0.0f ? 1.0f : -1.0f
			};

			contact.penetration = overlapY;
		}

		return contact;
	}

	void resolveContact(
		Body& a,
		Body& b,
		const Contact& contact,
		const PhysicsSettings& settings
	)
	{
		const float inverse_mass_summ = a.inverse_mass + b.inverse_mass;

		if (inverse_mass_summ <= 0.0f) {
			return;
		}
		
		const float corrected_depth = std::max(contact.penetration - settings.penetration_slop, 0.0f);

		const Vec2 correction =
			contact.normal * (corrected_depth * settings.position_correction / inverse_mass_summ);

		a.position += correction * a.inverse_mass;
		b.position += correction * (-b.inverse_mass);

		const Vec2 relative_velocity = a.velocity - b.velocity;
		const float regular_speed = dot(relative_velocity, contact.normal);

		if (regular_speed >= 0.0f) {
			return;
		}

		const float restitution = -regular_speed < settings.restitution_threshold ? 0.0f : settings.restitution;
		const float	impulse_magnitude = -(1.0f + restitution) * regular_speed / inverse_mass_summ;
		const Vec2 impulse = contact.normal * impulse_magnitude;

		a.velocity += impulse * a.inverse_mass;
		b.velocity += impulse * (-b.inverse_mass);

	}

	void resolveStaticContact(
		Body& moving,
		const Contact& contact,
		float restitution
	)
	{
		moving.position += contact.normal * contact.penetration;

		const float normal_speed = dot(moving.velocity, contact.normal);
		if (normal_speed >= 0.0f) {
			return;
		}
		moving.velocity += contact.normal * (-(1.0f + restitution) * normal_speed);
	}
}