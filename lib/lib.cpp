#include "lib.h"

int nEvents = 0;

Logger logger;

void Actor::eventNewRelativeCoordinates(float samplingrate,
	const vector<ObjectIdentifier>& objects) {

	AbstractTask::TaskResult tr = targetTask->robotExecutionStep(samplingrate, objects);
	saveMap(objects);
	nEvents++;
}

AbstractTask::TaskResult StraightTask::robotExecutionStep(float samplingrate,
	const vector<ObjectIdentifier>& objects) {

	newMotorAction();
	TaskResult tr;
	return tr;
}