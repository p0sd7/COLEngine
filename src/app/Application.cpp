#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>

#include "Application.h"

namespace
{
    struct WorldSnapshot
    {
        std::vector<Body> static_bodies;
        std::vector<Body> regular_bodies;
        std::vector<Body> fast_bodies;

        int ccd_events;
        bool ccd_limit;
        float ccd_discarded_time;
        int fast_substeps;
    };

    WorldSnapshot captureWorld(const PhysicsWorld& world)
    {
        return {
            world.getStaticBodies(),
            world.getRegularDynamicBodies(),
            world.getFastDynamicBodies(),

            world.lastCcdEvents(),
            world.ccdLimitReached(),
            world.lastCcdDiscardedTime(),
            world.lastFastSubsteps()
        };
    }

    std::string compareBodies(
        const std::vector<Body>& expected,
        const std::vector<Body>& actual,
        const std::string& group
    )
    {
        if (expected.size() != actual.size())
        {
            return group + ": body count differs";
        }

        for (std::size_t i = 0; i < expected.size(); ++i)
        {
            const Body& a = expected[i];
            const Body& b = actual[i];

            const std::string prefix =
                group + "[" + std::to_string(i)
                + "], id=" + std::to_string(a.id) + ": ";

            if (a.id != b.id)
            {
                return prefix + "id differs";
            }

            const auto compareFloat = [&](float x, float y,
                const char* field) -> std::string
                {
                    if (!std::isfinite(x) || !std::isfinite(y))
                    {
                        return prefix + field + " is not finite";
                    }

                    if (x != y)
                    {
                        return prefix + field + " differs";
                    }

                    return {};
                };

            const float valuesA[] = {
                a.position.x, a.position.y,
                a.velocity.x, a.velocity.y,
                a.half_size.x, a.half_size.y,
                a.inverse_mass
            };

            const float valuesB[] = {
                b.position.x, b.position.y,
                b.velocity.x, b.velocity.y,
                b.half_size.x, b.half_size.y,
                b.inverse_mass
            };

            const char* names[] = {
                "position.x", "position.y",
                "velocity.x", "velocity.y",
                "half_size.x", "half_size.y",
                "inverse_mass"
            };

            for (int field = 0; field < 7; ++field)
            {
                const std::string difference =
                    compareFloat(valuesA[field], valuesB[field], names[field]);

                if (!difference.empty())
                {
                    return difference;
                }
            }
        }

        return {};
    }

    std::string compareWorld(
        const WorldSnapshot& expected,
        const PhysicsWorld& actual
    )
    {
        std::string difference = compareBodies(
            expected.static_bodies,
            actual.getStaticBodies(),
            "static"
        );

        if (!difference.empty())
        {
            return difference;
        }

        difference = compareBodies(
            expected.regular_bodies,
            actual.getRegularDynamicBodies(),
            "regular"
        );

        if (!difference.empty())
        {
            return difference;
        }

        difference = compareBodies(
            expected.fast_bodies,
            actual.getFastDynamicBodies(),
            "fast"
        );

        if (!difference.empty())
        {
            return difference;
        }

        if (expected.ccd_events != actual.lastCcdEvents() ||
            expected.ccd_limit != actual.ccdLimitReached() ||
            expected.fast_substeps != actual.lastFastSubsteps())
        {
            return "Step counters differ";
        }

        const float discarded = actual.lastCcdDiscardedTime();

        if (!std::isfinite(expected.ccd_discarded_time) ||
            !std::isfinite(discarded) ||
            expected.ccd_discarded_time != discarded)
        {
            return "CCD discarded time differs or is not finite";
        }

        return {};
    }

    PhysicsSettings makePhysicsSettings(const RuntimeConfig& config)
    {
        PhysicsSettings settings{};

        settings.gravity = { 0.0f, config.gravity_y };

        settings.restitution = config.restitution;
        settings.solver_iterations = config.solver_iterations;

        settings.penetration_slop = config.penetration_slop;
        settings.position_correction = config.position_correction;
        settings.restitution_threshold = config.restitution_threshold;

        settings.max_ccd_events = config.max_ccd_events;
        settings.fast_substeps = config.fastSubsteps();

        settings.ccd_solver_iterations = config.ccd_solver_iterations;
        settings.contact_distance = config.contact_distance;
        settings.closing_speed_epsilon = config.closing_speed_epsilon;

        return settings;
    }
}

Application::Application(
    RuntimeConfig config,
    std::string configPath
) : config_(std::move(config)),
    config_path_(std::move(configPath)),
    world_(makePhysicsSettings(config_))
{
    resetScene();
}

void Application::stopRecording()
{
    if (!recording_)
    {
        return;
    }

    writeRecordingSample();

    if (!recording_)
    {
        return; 
    }

    recording_file_.close();
    recording_ = false;

    if (!recording_file_)
    {
        config_status_ = "Recording close ERROR";
        std::cerr << "Failed to finish " << recording_path_ << '\n';
        return;
    }

    const double meanMsPerFrame =
        recording_totals_.frames > 0
        ? recording_totals_.physics_ms
        / static_cast<double>(recording_totals_.frames)
        : 0.0;

    const double meanMsPerStep =
        recording_totals_.steps > 0
        ? recording_totals_.physics_ms
        / static_cast<double>(recording_totals_.steps)
        : 0.0;

    const double meanFastPerFrame =
        recording_totals_.frames > 0
        ? static_cast<double>(recording_totals_.fast_substeps)
        / static_cast<double>(recording_totals_.frames)
        : 0.0;

    config_status_ = "Recording saved: see console";

    std::cout
        << "\nFrame recording saved: " << recording_path_
        << "\nFrames: " << recording_totals_.frames
        << "\nPhysics steps: " << recording_totals_.steps
        << "\nMean physics ms/frame: " << meanMsPerFrame
        << "\nMean physics ms/step: " << meanMsPerStep
        << "\nMean fast steps/frame: " << meanFastPerFrame
        << '\n';
}

void Application::writeRecordingSample()
{
    if (!recording_ || stats_.frames == 0)
    {
        return;
    }

    const char* sceneName = "Unknown";

    switch (scene_)
    {
    case SceneType::Wall:
        sceneName = "Wall";
        break;

    case SceneType::MultipleBounces:
        sceneName = "MultipleBounces";
        break;

    case SceneType::Stack:
        sceneName = "Stack";
        break;

    case SceneType::System:
        sceneName = "System";
        break;
    }

    const char* modeName = "Unknown";

    switch (mode_)
    {
    case SimulationMode::Discrete:
        modeName = "Discrete";
        break;

    case SimulationMode::CCD:
        modeName = "CCD";
        break;

    case SimulationMode::MultiRate:
        modeName = "MultiRate";
        break;
    }

#ifdef NDEBUG
    const char* buildName = "Release";
#else
    const char* buildName = "Debug";
#endif

    const double msPerFrame =
        stats_.physics_ms / static_cast<double>(stats_.frames);

    const double msPerStep =
        stats_.steps > 0
        ? stats_.physics_ms / static_cast<double>(stats_.steps)
        : 0.0;

    const double fastPerFrame =
        static_cast<double>(stats_.fast_substeps)
        / static_cast<double>(stats_.frames);

    recording_file_
        << buildName << ','
        << sceneName << ','
        << modeName << ','
        << config_.physics_hz << ','
        << config_.fast_physics_hz << ','
        << config_.restitution << ','
        << stats_.elapsed << ','
        << stats_.frames << ','
        << stats_.steps << ','
        << stats_.physics_ms << ','
        << msPerFrame << ','
        << msPerStep << ','
        << stats_.fast_substeps << ','
        << fastPerFrame << ','
        << stats_.ccd_events << ','
        << stats_.ccd_limit_hits << ','
        << stats_.ccd_discarded_seconds * 1000.0
        << '\n';

    if (!recording_file_)
    {
        recording_ = false;
        recording_file_.close();

        config_status_ = "Recording write ERROR";
        std::cerr << "Failed to write " << recording_path_ << '\n';
        return;
    }

    recording_totals_.elapsed += stats_.elapsed;
    recording_totals_.frames += stats_.frames;
    recording_totals_.steps += stats_.steps;
    recording_totals_.physics_ms += stats_.physics_ms;
    recording_totals_.fast_substeps += stats_.fast_substeps;
    recording_totals_.ccd_events += stats_.ccd_events;
    recording_totals_.ccd_limit_hits += stats_.ccd_limit_hits;
    recording_totals_.ccd_discarded_seconds +=
        stats_.ccd_discarded_seconds;
}

void Application::startRecording()
{
    namespace fs = std::filesystem;

    try
    {
        const fs::path directory =
            fs::absolute(fs::path(config_path_)).parent_path()
            / "results";

        fs::create_directories(directory);

        const auto timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

        const fs::path path =
            directory /
            ("frames_" + std::to_string(timestamp) + ".csv");

        recording_file_.clear();
        recording_file_.open(path);

        if (!recording_file_)
        {
            throw std::runtime_error("Cannot create frame statistics CSV");
        }

        recording_file_.imbue(std::locale::classic());
        recording_file_ << std::fixed << std::setprecision(9);

        recording_file_
            << "build,scene,mode,physics_hz,fast_hz,restitution,"
            << "elapsed_seconds,frames,physics_steps,physics_ms,"
            << "ms_per_frame,ms_per_step,fast_substeps,"
            << "fast_steps_per_frame,ccd_events,ccd_limit_hits,"
            << "ccd_discarded_ms\n";

        recording_file_.flush();

        if (!recording_file_)
        {
            throw std::runtime_error("Cannot write CSV header");
        }

        recording_path_ = path.string();
        recording_totals_.reset();

        resetScene();

        stats_.reset();
        displayed_stats_ = {};

        recording_ = true;
        paused_ = false;

        config_status_ = "RECORDING: F8 to stop";

        std::cout
            << "\nFrame recording started: "
            << recording_path_ << '\n';
    }
    catch (const std::exception& error)
    {
        recording_ = false;

        if (recording_file_.is_open())
        {
            recording_file_.close();
        }

        config_status_ = "Recording ERROR: see console";
        std::cerr << error.what() << '\n';
    }
}

void Application::runBenchmark()
{
    using Clock = std::chrono::steady_clock;
    namespace fs = std::filesystem;

    constexpr int repeats = 5;
    constexpr int simulationSeconds = 10;

    const int steps = config_.physics_hz * simulationSeconds;
    const float dt = static_cast<float>(config_.fixedDt());

    struct SceneEntry
    {
        SceneType value;
        const char* name;
    };

    struct ModeEntry
    {
        SimulationMode value;
        const char* name;
    };

    const std::array<SceneEntry, 4> scenes{ {
        { SceneType::Wall, "Wall" },
        { SceneType::MultipleBounces, "MultipleBounces" },
        { SceneType::Stack, "Stack" },
        { SceneType::System, "System" }
    } };

    const std::array<ModeEntry, 3> modes{ {
        { SimulationMode::Discrete, "Discrete" },
        { SimulationMode::CCD, "CCD" },
        { SimulationMode::MultiRate, "MultiRate" }
    } };

#ifdef NDEBUG
    const char* buildType = "Release";
#else
    const char* buildType = "Debug";
#endif
    try
    {
        const fs::path outputDirectory =
            fs::absolute(fs::path(config_path_)).parent_path()
            / "results";

        fs::create_directories(outputDirectory);

        const auto timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

        const std::string runId = std::to_string(timestamp);

        const fs::path csvPath =
            outputDirectory / ("benchmark_" + runId + ".csv");

        const fs::path settingsPath =
            outputDirectory / ("benchmark_" + runId + "_settings.txt");

        std::ofstream settingsFile(settingsPath);

        if (!settingsFile)
        {
            throw std::runtime_error("Cannot create settings file");
        }

        settingsFile.imbue(std::locale::classic());
        settingsFile << std::setprecision(17);

        settingsFile
            << "build=" << buildType << '\n'
            << "repeats=" << repeats << '\n'
            << "simulation_seconds=" << simulationSeconds << '\n'
            << "physics_hz=" << config_.physics_hz << '\n'
            << "fast_physics_hz=" << config_.fast_physics_hz << '\n'
            << "restitution=" << config_.restitution << '\n'
            << "projectile_speed=" << config_.projectile_speed << '\n'
            << "gravity_y=" << config_.gravity_y << '\n'
            << "solver_iterations=" << config_.solver_iterations << '\n'
            << "ccd_solver_iterations="
            << config_.ccd_solver_iterations << '\n'
            << "max_ccd_events=" << config_.max_ccd_events << '\n'
            << "penetration_slop=" << config_.penetration_slop << '\n'
            << "position_correction="
            << config_.position_correction << '\n'
            << "restitution_threshold="
            << config_.restitution_threshold << '\n'
            << "contact_distance=" << config_.contact_distance << '\n'
            << "closing_speed_epsilon="
            << config_.closing_speed_epsilon << '\n'
            << "wall_thickness=" << config_.wall_thickness << '\n'
            << "stack_count=" << config_.stack_count << '\n'
            << "system_regular_count="
            << config_.system_regular_count << '\n'
            << "system_fast_count=" << config_.system_fast_count << '\n'
            << "system_obstacle_count="
            << config_.system_obstacle_count << '\n'
            << "system_regular_half_size="
            << config_.system_regular_half_size << '\n'
            << "system_fast_half_size="
            << config_.system_fast_half_size << '\n'
            << "max_frame_time=" << config_.max_frame_time << '\n';

        settingsFile.close();

        if (!settingsFile)
        {
            throw std::runtime_error("Failed to write settings file");
        }

        std::ofstream csv(csvPath);

        if (!csv)
        {
            throw std::runtime_error("Cannot create benchmark CSV");
        }

        csv.imbue(std::locale::classic());
        csv << std::fixed << std::setprecision(9);

        csv
            << "build,scene,mode,repeat,steps,simulated_seconds,"
            << "static_count,regular_count,fast_count,"
            << "total_physics_ms,mean_ms_per_step,"
            << "min_ms_per_step,max_ms_per_step,"
            << "fast_substeps,ccd_events,ccd_limit_hits,"
            << "ccd_discarded_ms\n";

        std::cout
            << "\nBenchmark started"
            << " | build=" << buildType
            << " | repeats=" << repeats
            << " | steps per run=" << steps
            << '\n';

        for (const SceneEntry& scene : scenes)
        {
            for (const ModeEntry& mode : modes)
            {
                {
                    PhysicsWorld warmup(makePhysicsSettings(config_));
                    SceneFactory::build(warmup, scene.value, config_);

                    for (int step = 0;
                        step < config_.physics_hz;
                        ++step)
                    {
                        warmup.step(dt, mode.value);
                    }
                }

                double sumOfRunMeans = 0.0;

                for (int repeat = 1; repeat <= repeats; ++repeat)
                {
                    PhysicsWorld world(makePhysicsSettings(config_));
                    SceneFactory::build(world, scene.value, config_);

                    double totalMs = 0.0;
                    double minMs = std::numeric_limits<double>::max();
                    double maxMs = 0.0;

                    std::uint64_t fastSubsteps = 0;
                    std::uint64_t ccdEvents = 0;
                    std::uint64_t ccdLimitHits = 0;

                    double discardedSeconds = 0.0;

                    for (int step = 0; step < steps; ++step)
                    {
                        const auto begin = Clock::now();

                        world.step(dt, mode.value);

                        const auto end = Clock::now();

                        const double elapsedMs =
                            std::chrono::duration<double, std::milli>(
                                end - begin
                            ).count();

                        totalMs += elapsedMs;
                        minMs = std::min(minMs, elapsedMs);
                        maxMs = std::max(maxMs, elapsedMs);

                        fastSubsteps += world.lastFastSubsteps();
                        ccdEvents += world.lastCcdEvents();

                        if (world.ccdLimitReached())
                        {
                            ++ccdLimitHits;
                        }

                        discardedSeconds +=
                            world.lastCcdDiscardedTime();
                    }

                    const double meanMs = totalMs / steps;
                    sumOfRunMeans += meanMs;

                    csv
                        << buildType << ','
                        << scene.name << ','
                        << mode.name << ','
                        << repeat << ','
                        << steps << ','
                        << steps * config_.fixedDt() << ','
                        << world.getStaticBodies().size() << ','
                        << world.getRegularDynamicBodies().size() << ','
                        << world.getFastDynamicBodies().size() << ','
                        << totalMs << ','
                        << meanMs << ','
                        << minMs << ','
                        << maxMs << ','
                        << fastSubsteps << ','
                        << ccdEvents << ','
                        << ccdLimitHits << ','
                        << discardedSeconds * 1000.0
                        << '\n';
                }

                std::cout
                    << scene.name << " / " << mode.name
                    << ": mean "
                    << sumOfRunMeans / repeats
                    << " ms/step\n";
            }
        }

        csv.close();

        if (!csv)
        {
            throw std::runtime_error("Failed to write benchmark CSV");
        }

        config_status_ = "Benchmark saved: see console";

        std::cout
            << "CSV: " << csvPath.string() << '\n'
            << "Settings: " << settingsPath.string() << '\n';
    }
    catch (const std::exception& error)
    {
        config_status_ = "Benchmark ERROR: see console";

        std::cerr
            << "Benchmark failed: "
            << error.what() << '\n';
    }
}

void Application::checkDeterminism()
{
    constexpr int stepCount = 600;

    const float dt = static_cast<float>(config_.fixedDt());

    try
    {
        std::cout
            << "\nDeterminism check"
            << " | scene=" << static_cast<int>(scene_)
            << " | mode=" << static_cast<int>(mode_)
            << " | steps=" << stepCount
            << " | simulated seconds=" << stepCount * config_.fixedDt()
            << '\n';

        std::vector<WorldSnapshot> reference;
        reference.reserve(stepCount + 1);

        std::uint64_t limitHits = 0;

        {
            PhysicsWorld first(makePhysicsSettings(config_));
            SceneFactory::build(first, scene_, config_);

            reference.push_back(captureWorld(first));

            for (int step = 1; step <= stepCount; ++step)
            {
                first.step(dt, mode_);

                if (first.ccdLimitReached())
                {
                    ++limitHits;
                }

                reference.push_back(captureWorld(first));
            }
        }

        PhysicsWorld second(makePhysicsSettings(config_));
        SceneFactory::build(second, scene_, config_);

        for (int step = 0; step <= stepCount; ++step)
        {
            if (step > 0)
            {
                second.step(dt, mode_);
            }

            const std::string difference =
                compareWorld(reference[step], second);

            if (!difference.empty())
            {
                config_status_ =
                    "Repeatability FAIL at step " + std::to_string(step);

                std::cout
                    << "FAIL at step " << step
                    << ": " << difference << '\n';

                return;
            }
        }

        config_status_ = "Repeatability PASS";

        std::cout
            << "PASS: states match at every step.\n"
            << "CCD limit hits in each run: " << limitHits
            << '\n';
    }
    catch (const std::exception& error)
    {
        config_status_ = "Repeatability ERROR: see console";

        std::cerr
            << "Determinism check error: "
            << error.what() << '\n';
    }
}

void Application::executePhysicsStep()
{
    const float dt = static_cast<float>(config_.fixedDt());

    const auto begin = std::chrono::steady_clock::now();

    world_.step(dt, mode_);

    const auto end = std::chrono::steady_clock::now();

    const double durationMs =
        std::chrono::duration<double, std::milli>(
            end - begin
        ).count();

    stats_.physics_ms += durationMs;
    ++stats_.steps;

    stats_.fast_substeps += world_.lastFastSubsteps();
    stats_.ccd_events += world_.lastCcdEvents();

    if (world_.ccdLimitReached())
    {
        ++stats_.ccd_limit_hits;
    }

    stats_.ccd_discarded_seconds +=
        world_.lastCcdDiscardedTime();
}

void Application::resetScene()
{
    SceneFactory::build(world_, scene_, config_);

    paused_ = true;
    accumulator_ = 0.0;
}

bool Application::reloadConfig()
{
    try
    {
        RuntimeConfig candidate = RuntimeConfig::load(config_path_);

        PhysicsWorld replacement(makePhysicsSettings(candidate));

        SceneFactory::build(replacement, scene_, candidate);

        world_ = std::move(replacement);
        config_ = std::move(candidate);

        paused_ = true;
        accumulator_ = 0.0;

        config_status_ = "Config loaded";
        std::cout << "Config loaded: " << config_path_ << '\n';

        return true;
    }
    catch (const std::exception& error)
    {
        config_status_ = "CONFIG ERROR: see console";

        std::cerr
            << "Config was not applied: "
            << error.what()
            << '\n';

        return false;
    }
}

void Application::run()
{
	using clock = std::chrono::steady_clock;
	auto previous_time = clock::now();

    while (frontend_.isOpen())
    {

		auto current_time = clock::now();
        double frame_time = std::chrono::duration<double>(current_time - previous_time).count();

        previous_time = current_time;
        frame_time = std::min(frame_time, config_.max_frame_time);

		const auto commands = frontend_.processEvents();


        if (!frontend_.isOpen())
        {
            stopRecording();
            break;
        }

		bool skip_elapsed_time = false;

        for (const AppCommand command : commands) {
            if (recording_ &&
                command != AppCommand::ToggleRecording &&
                command != AppCommand::SingleStep)
            {
                stopRecording();
            }
            switch (command) {
			case AppCommand::TogglePause:
				paused_ = !paused_;
				accumulator_ = 0.0;
				skip_elapsed_time = true;
				break;

            case AppCommand::SingleStep:
                if (paused_)
                {
                    executePhysicsStep();
                }
                break;

			case AppCommand::Reset:
				resetScene();
				skip_elapsed_time = true;
                break;

            case AppCommand::SelectDiscrete:
                mode_ = SimulationMode::Discrete;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::SelectCCD:
                mode_ = SimulationMode::CCD;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::SelectMultiRate:
                mode_ = SimulationMode::MultiRate;
                resetScene();
                skip_elapsed_time = true;
                break;
            case AppCommand::SelectWallScene:
                scene_ = SceneType::Wall;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::SelectBouncesScene:
                scene_ = SceneType::MultipleBounces;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::SelectStackScene:
                scene_ = SceneType::Stack;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::SelectSystemScene:
                scene_ = SceneType::System;
                resetScene();
                skip_elapsed_time = true;
                break;

            case AppCommand::ReloadConfig:
                if (reloadConfig())
                {
                    skip_elapsed_time = true;
                }
                else
                {
                    previous_time = clock::now();
                    frame_time = 0.0;
                }
                break;

            case AppCommand::CheckDeterminism:
                paused_ = true;
                accumulator_ = 0.0;

                checkDeterminism();

                skip_elapsed_time = true;
                break;

            case AppCommand::RunBenchmark:
                paused_ = true;
                accumulator_ = 0.0;

                runBenchmark();

                skip_elapsed_time = true;
                break;

            case AppCommand::ToggleRecording:
                if (recording_)
                {
                    stopRecording();

                    paused_ = true;
                    accumulator_ = 0.0;
                }
                else
                {
                    startRecording();
                }

                skip_elapsed_time = true;
                break;

            default:
                break;
            }
        }

        if (skip_elapsed_time)
        {
            previous_time = clock::now();
            frame_time = 0.0;

            stats_.reset();
            displayed_stats_ = {};
        }

        if (!paused_ && !skip_elapsed_time)
        {
            accumulator_ += frame_time;

            while (accumulator_ >= config_.fixedDt())
            {
                executePhysicsStep();
                accumulator_ -= config_.fixedDt();
            }
        }

        std::string scene_name;

        switch (scene_)
        {
        case SceneType::Wall:
            scene_name = "Wall";
            break;

        case SceneType::MultipleBounces:
            scene_name = "Multiple bounces";
            break;

        case SceneType::Stack:
            scene_name = "Stack";
            break;

        case SceneType::System:
            scene_name = "System";
            break;
        }

        const bool measuredFrame =
            !skip_elapsed_time &&
            (!paused_ || stats_.steps > 0);

        if (measuredFrame)
        {
            ++stats_.frames;
            stats_.elapsed += frame_time;

            const bool publish =
                stats_.elapsed >= 0.5 ||
                (paused_ && stats_.steps > 0);

            if (publish)
            {
                displayed_stats_.ms_per_step =
                    stats_.steps > 0
                    ? stats_.physics_ms / static_cast<double>(stats_.steps)
                    : 0.0;

                displayed_stats_.ms_per_frame =
                    stats_.frames > 0
                    ? stats_.physics_ms / static_cast<double>(stats_.frames)
                    : 0.0;

                displayed_stats_.fast_steps_per_frame =
                    stats_.frames > 0
                    ? static_cast<double>(stats_.fast_substeps)
                    / static_cast<double>(stats_.frames)
                    : 0.0;

                displayed_stats_.ccd_events = stats_.ccd_events;
                displayed_stats_.ccd_limit_hits = stats_.ccd_limit_hits;

                displayed_stats_.ccd_discarded_ms =
                    stats_.ccd_discarded_seconds * 1000.0;

                writeRecordingSample();
                stats_.reset();
            }
        }

        scene_name += " | Hz=";
        scene_name += std::to_string(config_.physics_hz);
        scene_name += "/";
        scene_name += std::to_string(config_.fast_physics_hz);

        scene_name += " | e=";
        scene_name += std::to_string(config_.restitution);

if (!config_status_.empty())
{
    scene_name += " | ";
    scene_name += config_status_;
}

        frontend_.setSimulationTitle(
            mode_,
            paused_,
            displayed_stats_,
            scene_name
        );

        frontend_.render(world_);

    }
}