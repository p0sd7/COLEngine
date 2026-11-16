#ifndef SFMLFRONTEND_H
#define SFMLFRONTEND_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>

#include "../app/AppCommand.h"
#include "../physics/PhysicsSettings.h"
#include "../app/PhysicStats.h"

class PhysicsWorld;

class SfmlFrontend {
public:
	SfmlFrontend();

	std::vector<AppCommand> processEvents();
	void render(const PhysicsWorld& world);
	bool isOpen() const;
	void setSimulationTitle(
		SimulationMode mode,
		bool paused,
		const PhysicStatsView& stats,
		const std::string& scene_name
	);
private:
	sf::RenderWindow window_;
};

#endif // !SFMLFRONTEND_H
