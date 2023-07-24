#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) {

	AbstractTask::TaskResult tr = targetTask->taskExecutionStep(samplingrate, obs);
	saveMap(obs);
	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs) {

	eventNewMotorAction();
	TaskResult tr;
	return tr;
}