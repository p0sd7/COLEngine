#include "Config.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace
{
    std::string trim(const std::string& text)
    {
        const auto first = text.find_first_not_of(" \t\r\n");

        if (first == std::string::npos)
        {
            return {};
        }

        const auto last = text.find_last_not_of(" \t\r\n");

        return text.substr(first, last - first + 1);
    }

    template<typename T>
    void parseNumber(const std::string& text, T& destination)
    {
        std::istringstream input(text);
        T value{};

        if (!(input >> value))
        {
            throw std::runtime_error("Expected a number");
        }

        input >> std::ws;

        if (!input.eof())
        {
            throw std::runtime_error(
                "Unexpected characters after the number"
            );
        }

        if (!std::isfinite(static_cast<double>(value)))
        {
            throw std::runtime_error("Number must be finite");
        }

        destination = value;
    }
}

void RuntimeConfig::validate() const
{
    const auto require = [](bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        };

    require(
        physics_hz >= 1 && physics_hz <= 1000,
        "physics_hz must be in [1, 1000]"
    );

    require(
        fast_physics_hz >= physics_hz &&
        fast_physics_hz <= 10000,
        "fast_physics_hz must be in [physics_hz, 10000]"
    );

    require(
        fast_physics_hz % physics_hz == 0,
        "fast_physics_hz must be divisible by physics_hz"
    );

    require(
        restitution >= 0.0f && restitution <= 1.0f,
        "restitution must be in [0, 1]"
    );

    require(
        projectile_speed > 0.0f,
        "projectile_speed must be positive"
    );

    require(gravity_y >= 0.0f, "gravity_y must be non-negative");

    require(
        solver_iterations >= 1 && solver_iterations <= 1000,
        "solver_iterations must be in [1, 1000]"
    );

    require(
        ccd_solver_iterations >= 1 &&
        ccd_solver_iterations <= 1000,
        "ccd_solver_iterations must be in [1, 1000]"
    );

    require(
        max_ccd_events >= 1 && max_ccd_events <= 10000,
        "max_ccd_events must be in [1, 10000]"
    );

    require(
        penetration_slop >= 0.0f,
        "penetration_slop must be non-negative"
    );

    require(
        position_correction > 0.0f &&
        position_correction <= 1.0f,
        "position_correction must be in (0, 1]"
    );

    require(
        restitution_threshold >= 0.0f,
        "restitution_threshold must be non-negative"
    );

    require(
        contact_distance > 0.0f &&
        contact_distance >= penetration_slop,
        "contact_distance must be positive and >= penetration_slop"
    );

    require(
        closing_speed_epsilon > 0.0f,
        "closing_speed_epsilon must be positive"
    );

    require(wall_thickness > 0.0f, "wall_thickness must be positive");

    require(
        stack_count >= 1 && stack_count <= 10,
        "stack_count must be in [1, 10]"
    );

    require(
        system_regular_count >= 10 && system_regular_count <= 100,
        "system_regular_count must be in [10, 100]"
    );

    require(
        system_fast_count >= 1 && system_fast_count <= 10,
        "system_fast_count must be in [1, 10]"
    );

    require(
        system_obstacle_count >= 2 && system_obstacle_count <= 9,
        "system_obstacle_count must be in [2, 9], plus one floor"
    );

    require(
        system_regular_half_size > 0.0f &&
        system_fast_half_size > 0.0f,
        "Body half sizes must be positive"
    );

    require(
        max_frame_time > 0.0 && max_frame_time <= 1.0,
        "max_frame_time must be in (0, 1]"
    );
}

RuntimeConfig RuntimeConfig::load(const std::string& path)
{
    std::ifstream file(path);

    if (!file)
    {
        throw std::runtime_error("Cannot open config: " + path);
    }

    RuntimeConfig result;

    using Setter = std::function<void(const std::string&)>;
    std::unordered_map<std::string, Setter> setters;

    const auto bind = [&setters](const char* name, auto& field)
        {
            auto* destination = &field;

            setters.emplace(
                name,
                [destination](const std::string& value)
                {
                    parseNumber(value, *destination);
                }
            );
        };

    bind("physics_hz", result.physics_hz);
    bind("fast_physics_hz", result.fast_physics_hz);
    bind("restitution", result.restitution);
    bind("projectile_speed", result.projectile_speed);
    bind("gravity_y", result.gravity_y);

    bind("solver_iterations", result.solver_iterations);
    bind("ccd_solver_iterations", result.ccd_solver_iterations);
    bind("max_ccd_events", result.max_ccd_events);

    bind("penetration_slop", result.penetration_slop);
    bind("position_correction", result.position_correction);
    bind("restitution_threshold", result.restitution_threshold);
    bind("contact_distance", result.contact_distance);
    bind("closing_speed_epsilon", result.closing_speed_epsilon);

    bind("wall_thickness", result.wall_thickness);
    bind("stack_count", result.stack_count);
    bind("system_regular_count", result.system_regular_count);
    bind("system_fast_count", result.system_fast_count);
    bind("system_obstacle_count", result.system_obstacle_count);
    bind("system_regular_half_size", result.system_regular_half_size);
    bind("system_fast_half_size", result.system_fast_half_size);
    bind("max_frame_time", result.max_frame_time);

    std::unordered_set<std::string> seen;

    std::string line;
    int lineNumber = 0;

    while (std::getline(file, line))
    {
        ++lineNumber;

        if (lineNumber == 1 &&
            line.compare(0, 3, "\xEF\xBB\xBF") == 0)
        {
            line.erase(0, 3);
        }

        const auto comment = line.find_first_of("#;");

        if (comment != std::string::npos)
        {
            line.erase(comment);
        }

        line = trim(line);

        if (line.empty())
        {
            continue;
        }

        try
        {
            const auto separator = line.find('=');

            if (separator == std::string::npos)
            {
                throw std::runtime_error("Expected key=value");
            }

            const std::string key = trim(line.substr(0, separator));
            const std::string value = trim(line.substr(separator + 1));

            const auto setter = setters.find(key);

            if (setter == setters.end())
            {
                throw std::runtime_error("Unknown setting: " + key);
            }

            if (!seen.insert(key).second)
            {
                throw std::runtime_error("Duplicate setting: " + key);
            }

            setter->second(value);
        }
        catch (const std::exception& error)
        {
            throw std::runtime_error(
                path + ":" + std::to_string(lineNumber)
                + ": " + error.what()
            );
        }
    }

    if (file.bad())
    {
        throw std::runtime_error("Error reading config: " + path);
    }

    result.validate();
    return result;
}