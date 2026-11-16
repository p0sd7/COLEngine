#include <iomanip>
#include <sstream>

#include "SfmlFrontend.h"
#include "../physics/Body.h"
#include "../physics/PhysicsWorld.h"

SfmlFrontend::SfmlFrontend() : window_(sf::VideoMode({1280u, 720u}), "Physics Processes Modeling") {
	window_.setVerticalSyncEnabled(true);
	window_.setKeyRepeatEnabled(false);
}

void SfmlFrontend::setSimulationTitle(
	SimulationMode mode,
	bool paused,
	const PhysicStatsView& stats,
	const std::string& scene_name
)
{
	std::ostringstream title;

	title << "Mechanics Lab | " << scene_name << " | ";

	switch (mode)
	{
	case SimulationMode::Discrete:
		title << "Discrete";
		break;

	case SimulationMode::CCD:
		title << "CCD";
		break;

	case SimulationMode::MultiRate:
		title << "Multi-rate";
		break;
	}

	title << (paused ? " | PAUSED" : " | RUNNING");

	title << std::fixed << std::setprecision(4);

	title << " | ms/step=" << stats.ms_per_step;
	title << " | ms/frame=" << stats.ms_per_frame;

	title << std::setprecision(2);
	title << " | fast/frame=" << stats.fast_steps_per_frame;

	if (mode == SimulationMode::CCD)
	{
		title << " | CCD events=" << stats.ccd_events;
		title << " | limits=" << stats.ccd_limit_hits;
		title << " | lost ms=" << stats.ccd_discarded_ms;
	}

	window_.setTitle(title.str());
}
bool SfmlFrontend::isOpen() const {
	return window_.isOpen();
}

std::vector<AppCommand> SfmlFrontend::processEvents() {
	std::vector<AppCommand> commands;
	while (const auto event = window_.pollEvent()) {
		if (event->is<sf::Event::Closed>()) {
			window_.close();
			break;
		}

		if (const auto* key = event->getIf<sf::Event::KeyPressed>())
		{
			switch (key->scancode) {
			case sf::Keyboard::Scancode::Escape:
				window_.close();
				break;

			case sf::Keyboard::Scancode::Space:
				commands.push_back(AppCommand::TogglePause);
				break;

			case sf::Keyboard::Scancode::N:
				commands.push_back(AppCommand::SingleStep);
				break;

			case sf::Keyboard::Scancode::R:
				commands.push_back(AppCommand::Reset);
				break;

			case sf::Keyboard::Scancode::Num1:
				commands.push_back(AppCommand::SelectDiscrete);
				break;

			case sf::Keyboard::Scancode::Num2:
				commands.push_back(AppCommand::SelectCCD);
				break;

			case sf::Keyboard::Scancode::Num3:
				commands.push_back(AppCommand::SelectMultiRate);
				break;

			case sf::Keyboard::Scancode::F1:
				commands.push_back(AppCommand::SelectWallScene);
				break;

			case sf::Keyboard::Scancode::F2:
				commands.push_back(AppCommand::SelectBouncesScene);
				break;

			case sf::Keyboard::Scancode::F3:
				commands.push_back(AppCommand::SelectStackScene);
				break;

			case sf::Keyboard::Scancode::F4:
				commands.push_back(AppCommand::SelectSystemScene);
				break;

			case sf::Keyboard::Scancode::F5:
				commands.push_back(AppCommand::ReloadConfig);
				break;

			case sf::Keyboard::Scancode::F6:
				commands.push_back(AppCommand::CheckDeterminism);
				break;

			case sf::Keyboard::Scancode::F7:
				commands.push_back(AppCommand::RunBenchmark);
				break;

			case sf::Keyboard::Scancode::F8:
				commands.push_back(AppCommand::ToggleRecording);
				break;

			default:
				break;
			}
		}
	}
	return commands;
}

void SfmlFrontend::render(const PhysicsWorld& world) {
	window_.clear(sf::Color::White);

	const auto drawBodies = [this](const std::vector<Body>& bodies, sf::Color color) {
		for (const Body& body : bodies) {
			sf::RectangleShape rectangle;

			rectangle.setSize(sf::Vector2f{
				body.half_size.x * 2.0f, body.half_size.y * 2.0f
			});

			rectangle.setOrigin(sf::Vector2f{
				body.half_size.x,
				body.half_size.y
				});

			rectangle.setPosition(sf::Vector2f{
				body.position.x,
				body.position.y
				});

			rectangle.setFillColor(color);

			window_.draw(rectangle);
		}};

	drawBodies(world.getStaticBodies(), sf::Color(140, 145, 155));
	drawBodies(world.getRegularDynamicBodies(), sf::Color(80, 170, 240));
	drawBodies(world.getFastDynamicBodies(), sf::Color(250, 170, 60));

	window_.display();
}