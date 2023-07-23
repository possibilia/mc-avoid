#include "lib.h"

int nEvents = 0;

Logger logger;

void Actor::eventNewRelativeCoordinates(float samplingrate,
	const vector<ObjectIdentifier>& objects) {

	AbstractTask::TaskResult tr = targetTask->robotExecutionStep(samplingrate, objects);
	logger.printf("Task result = %d\n", tr);
}

AbstractTask::TaskResult StraightTask::robotExecutionStep(float samplingrate,
	const vector<ObjectIdentifier>& objects) {

	float d = 6.0;
	ObjectIdentifier nearest;
	ObjectIdentifier origin(0.0f, 0.0f);

	for (unsigned i = 0; i < objects.size(); i++) {
		if (objects[i].isValid()) {
			float tmp = objects[i].getDistance(origin);
			if (tmp < d) {
				d = tmp;
				nearest = objects[i];
			}
		}
	}

	logger.printf("Nearest object %f [m]", nearest);

	newMotorAction();
	TaskResult tr;
	if (nearest.getDistance(origin) < 1.0) {
		tr.setDisturbance(nearest);
	}
	return tr;
}