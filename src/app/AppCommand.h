#ifndef APPCOMMAND_H
#define APPCOMMAND_H

enum class AppCommand {
	TogglePause,
	SingleStep,
	Reset,
	SelectDiscrete,
	SelectCCD,
	SelectMultiRate,
	SelectWallScene,
	SelectBouncesScene,
	SelectStackScene,
	SelectSystemScene,
	ReloadConfig,
	CheckDeterminism,
	RunBenchmark,
	ToggleRecording
};

#endif // APPCOMMAND_H