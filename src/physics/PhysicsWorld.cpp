#include <cstddef>
#include <algorithm>
#include <optional>
#include <stdexcept>

#include "PhysicsWorld.h"
#include "Collision.h"


PhysicsWorld::PhysicsWorld(PhysicsSettings settings)
	: settings_(settings)
{
}

void PhysicsWorld::solveCcdContacts(
    const std::vector<Body*>& bodies
)
{
    struct Constraint
    {
        Body* a;
        Body* b;

        Vec2 normal;

        float targetSpeed;
        float accumulatedImpulse;
    };

    std::vector<Constraint> contacts;

    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
        {
            Body& a = *bodies[i];
            Body& b = *bodies[j];

            if (a.inverse_mass + b.inverse_mass <= 0.0f)
            {
                continue;
            }

            const auto contact = Collision::findTouchingContact(
                    a,
                    b,
                    settings_.contact_distance
                );

            if (!contact)
            {
                continue;
            }

            const float normalSpeed = dot(
                a.velocity - b.velocity,
                contact->normal
            );

            float targetSpeed = 0.0f;

            if (normalSpeed < -settings_.restitution_threshold)
            {
                targetSpeed =
                    -settings_.restitution * normalSpeed;
            }

            contacts.push_back({
                &a,
                &b,
                contact->normal,
                targetSpeed,
                0.0f
                });
        }
    }

    const int iterations = settings_.ccd_solver_iterations;

    for (int iteration = 0; iteration < iterations; ++iteration)
    {
        for (Constraint& contact : contacts)
        {
            Body& a = *contact.a;
            Body& b = *contact.b;

            const float inverseMassSum =
                a.inverse_mass + b.inverse_mass;

            const float normalSpeed = dot(
                a.velocity - b.velocity,
                contact.normal
            );

            const float impulseChange =
                (contact.targetSpeed - normalSpeed)
                / inverseMassSum;

            const float oldImpulse = contact.accumulatedImpulse;

            contact.accumulatedImpulse = std::max(
                oldImpulse + impulseChange,
                0.0f
            );

            const float appliedImpulse =
                contact.accumulatedImpulse - oldImpulse;

            const Vec2 impulse =
                contact.normal * appliedImpulse;

            a.velocity += impulse * a.inverse_mass;
            b.velocity += impulse * (-b.inverse_mass);
        }
    }

    for (int iteration = 0;
        iteration < settings_.solver_iterations;
        ++iteration)
    {
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            for (std::size_t j = i + 1; j < bodies.size(); ++j)
            {
                Body& a = *bodies[i];
                Body& b = *bodies[j];

                const float inverseMassSum =
                    a.inverse_mass + b.inverse_mass;

                if (inverseMassSum <= 0.0f)
                {
                    continue;
                }

                const auto contact =
                    Collision::findContact(a, b);

                if (!contact)
                {
                    continue;
                }

                const float depth = std::max(
                    contact->penetration
                    - settings_.penetration_slop,
                    0.0f
                );

                const Vec2 correction =
                    contact->normal
                    * (depth * settings_.position_correction
                        / inverseMassSum);

                a.position += correction * a.inverse_mass;
                b.position += correction * (-b.inverse_mass);
            }
        }
    }
}

void PhysicsWorld::setGravity(Vec2 gravity)
{
    settings_.gravity = gravity;
}

std::vector<Body*> PhysicsWorld::collectBodies()
{
    std::vector<Body*> bodies;

    bodies.reserve(
        static_bodies_.size()
        + regular_dynamic_bodies_.size()
        + fast_dynamic_bodies_.size()
    );

    for (Body& body : static_bodies_)
    {
        bodies.push_back(&body);
    }

    for (Body& body : regular_dynamic_bodies_)
    {
        bodies.push_back(&body);
    }

    for (Body& body : fast_dynamic_bodies_)
    {
        bodies.push_back(&body);
    }

    return bodies;
}

void PhysicsWorld::applyGravity(float dt)
{
    for (Body& body : regular_dynamic_bodies_)
    {
        body.velocity += settings_.gravity * dt;
    }

    for (Body& body : fast_dynamic_bodies_)
    {
        body.velocity += settings_.gravity * dt;
    }
}

void PhysicsWorld::advanceBodies(float dt)
{
    for (Body& body : regular_dynamic_bodies_)
    {
        body.position += body.velocity * dt;
    }

    for (Body& body : fast_dynamic_bodies_)
    {
        body.position += body.velocity * dt;
    }
}

void PhysicsWorld::solveOverlaps(
    const std::vector<Body*>& bodies
)
{
    for (int iteration = 0;
        iteration < settings_.solver_iterations;
        ++iteration)
    {
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            for (std::size_t j = i + 1; j < bodies.size(); ++j)
            {
                Body& a = *bodies[i];
                Body& b = *bodies[j];

                if (a.inverse_mass + b.inverse_mass <= 0.0f)
                {
                    continue;
                }

                const auto contact = Collision::findContact(a, b);

                if (contact)
                {
                    Collision::resolveContact(
                        a, b, *contact, settings_
                    );
                }
            }
        }
    }
}

void PhysicsWorld::step(float dt, SimulationMode mode)
{
    last_ccd_events_ = 0;
    ccd_limit_reached_ = false;
    last_fast_substeps_ = 0;
    last_ccd_discarded_time_ = 0.0f;

    if (dt <= 0.0f)
    {
        return;
    }

    switch (mode)
    {
    case SimulationMode::Discrete:
        stepDiscrete(dt);
        break;

    case SimulationMode::CCD:
        stepCCD(dt);
        break;

    case SimulationMode::MultiRate:
        stepMultiRate(dt);
        break;
    }
}

void PhysicsWorld::stepDiscrete(float dt)
{
    const auto bodies = collectBodies();

    applyGravity(dt);
    advanceBodies(dt);
    solveOverlaps(bodies);
}

void PhysicsWorld::stepCCD(float dt)
{
    const auto bodies = collectBodies();

    applyGravity(dt);

    solveCcdContacts(bodies);

    float remainingTime = dt;

    while (remainingTime > 0.0f)
    {
        std::optional<Collision::SweepHit> earliestHit;

        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            for (std::size_t j = i + 1; j < bodies.size(); ++j)
            {
                Body& a = *bodies[i];
                Body& b = *bodies[j];

                if (a.inverse_mass + b.inverse_mass <= 0.0f)
                {
                    continue;
                }

                const auto hit =
                    Collision::sweepAabb(a, b, remainingTime);

                if (!hit)
                {
                    continue;
                }

                const float closingSpeed = dot(
                    a.velocity - b.velocity,
                    hit->normal
                );

                if (closingSpeed >= -settings_.closing_speed_epsilon)
                {
                    continue;
                }

                if (!earliestHit || hit->time < earliestHit->time)
                {
                    earliestHit = hit;
                }
            }
        }

        if (!earliestHit)
        {
            advanceBodies(remainingTime);
            remainingTime = 0.0f;
            break;
        }

        if (last_ccd_events_ >= settings_.max_ccd_events)
        {
            ccd_limit_reached_ = true;
            last_ccd_discarded_time_ = remainingTime;
            break;
        }

        advanceBodies(earliestHit->time);

        remainingTime = std::max(
            0.0f,
            remainingTime - earliestHit->time
        );


        solveCcdContacts(bodies);

        ++last_ccd_events_;
    }

    solveCcdContacts(bodies);
}
void PhysicsWorld::stepMultiRate(float dt)
{
    if (settings_.fast_substeps < 1)
    {
        throw std::logic_error(
            "Multi-rate requires at least one fast substep"
        );
    }

    const auto collide = [this](Body& a, Body& b)
        {
            const auto contact = Collision::findContact(a, b);

            if (contact)
            {
                Collision::resolveContact(
                    a,
                    b,
                    *contact,
                    settings_
                );
            }
        };

    for (Body& body : regular_dynamic_bodies_)
    {
        body.velocity += settings_.gravity * dt;
    }

    std::vector<Body> anchors = regular_dynamic_bodies_;

    std::vector<float> anchorTimes(
        regular_dynamic_bodies_.size(),
        0.0f
    );

    std::vector<Body> predicted = regular_dynamic_bodies_;

    const float fastDt =
        dt / static_cast<float>(settings_.fast_substeps);

    for (int substep = 0;
        substep < settings_.fast_substeps;
        ++substep)
    {

        const float currentTime =
            substep + 1 == settings_.fast_substeps
            ? dt
            : fastDt * static_cast<float>(substep + 1);

        for (Body& body : fast_dynamic_bodies_)
        {
            body.velocity += settings_.gravity * fastDt;
            body.position += body.velocity * fastDt;
        }

        for (std::size_t i = 0; i < anchors.size(); ++i)
        {
            predicted[i] = anchors[i];

            const float elapsed =
                currentTime - anchorTimes[i];

            predicted[i].position +=
                predicted[i].velocity * elapsed;
        }

        const std::vector<Body> beforeContacts = predicted;

        for (int iteration = 0;
            iteration < settings_.solver_iterations;
            ++iteration)
        {
            for (Body& fast : fast_dynamic_bodies_)
            {
                for (Body& obstacle : static_bodies_)
                {
                    collide(fast, obstacle);
                }
            }

            for (std::size_t i = 0;
                i < fast_dynamic_bodies_.size();
                ++i)
            {
                for (std::size_t j = i + 1;
                    j < fast_dynamic_bodies_.size();
                    ++j)
                {
                    collide(
                        fast_dynamic_bodies_[i],
                        fast_dynamic_bodies_[j]
                    );
                }
            }

            for (Body& fast : fast_dynamic_bodies_)
            {
                for (Body& regular : predicted)
                {
                    collide(fast, regular);
                }
            }
        }

        for (std::size_t i = 0; i < predicted.size(); ++i)
        {
            const Body& before = beforeContacts[i];
            const Body& after = predicted[i];

            const bool changed =
                before.position.x != after.position.x ||
                before.position.y != after.position.y ||
                before.velocity.x != after.velocity.x ||
                before.velocity.y != after.velocity.y;

            if (changed)
            {
                anchors[i] = after;
                anchorTimes[i] = currentTime;
            }
        }

        ++last_fast_substeps_;
    }

    for (std::size_t i = 0;
        i < regular_dynamic_bodies_.size();
        ++i)
    {
        regular_dynamic_bodies_[i] = anchors[i];

        regular_dynamic_bodies_[i].position +=
            anchors[i].velocity * (dt - anchorTimes[i]);
    }

    const auto bodies = collectBodies();
    solveOverlaps(bodies);
}

void PhysicsWorld::clear() {
	static_bodies_.clear();
	regular_dynamic_bodies_.clear();
	fast_dynamic_bodies_.clear();
	next_body_id_ = 1;
    last_ccd_events_ = 0;
    ccd_limit_reached_ = false;
    last_fast_substeps_ = 0;
    last_ccd_discarded_time_ = 0.0f;
}

void PhysicsWorld::addStatic(const Body& body) {
	Body new_body = body;
	new_body.id = next_body_id_++;
	new_body.inverse_mass = 0.0f;
	new_body.velocity = { 0.0f, 0.0f };
	static_bodies_.push_back(new_body);
}

void PhysicsWorld::addRegularDynamic(const Body& body) {
	Body new_body = body;
	new_body.id = next_body_id_++;
	regular_dynamic_bodies_.push_back(new_body);
}

void PhysicsWorld::addFastDynamic(const Body& body) {
	Body new_body = body;
	new_body.id = next_body_id_++;
	fast_dynamic_bodies_.push_back(new_body);
}


const std::vector<Body>& PhysicsWorld::getStaticBodies() const {
	return static_bodies_;
}

const std::vector<Body>& PhysicsWorld::getRegularDynamicBodies() const {
	return regular_dynamic_bodies_;
}

const std::vector<Body>& PhysicsWorld::getFastDynamicBodies() const {
	return fast_dynamic_bodies_;
}