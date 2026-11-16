#ifndef APPLICATION_H
#define APPLICATION_H

#include "../platform/SfmlFrontend.h"
#include "../physics/PhysicsWorld.h"
#include "../scenes/SceneFactory.h"
#include "../configure/Config.h"
#include "PhysicStats.h"
#include <string>
#include <fstream>

class Application {
public:
	Application(
		RuntimeConfig config,
		std::string config_path
	);
	void run();
private:
	void resetScene();
	bool reloadConfig();
	void executePhysicsStep();
	void checkDeterminism();
	void runBenchmark();

	RuntimeConfig config_;
	std::string config_path_;
	std::string config_status_;
	
	SceneType scene_ = SceneType::Wall;
	SfmlFrontend frontend_;
	PhysicsWorld world_;
	SimulationMode mode_ = SimulationMode::Discrete;

	bool paused_ = true;
	double accumulator_ = 0.0;
	
	PhysicStats stats_;
	PhysicStatsView displayed_stats_;

	void startRecording();
	void stopRecording();
	void writeRecordingSample();

	bool recording_ = false;
	std::ofstream recording_file_;
	std::string recording_path_;

	PhysicStats recording_totals_;
	
};

#endif // !APPLICATION_H
