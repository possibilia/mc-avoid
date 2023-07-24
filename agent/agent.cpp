#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) {

	shared_ptr<AbstractTask> left = make_shared<Rotate90Left>();
	shared_ptr<AbstractTask> right = make_shared<Rotate90Right>();
	shared_ptr<AbstractTask> straight = make_shared<StraightTask>();

	AbstractTask::TaskResult tr;
	targetTask->taskExecutionStep(samplingrate, obs);

	saveMap(obs);
	nEvents++;

	if (nEvents == 20) {
		left->init(targetTask);
		targetTask = left;
	}

	if (nEvents == 40) {
		right->init(targetTask);
		targetTask = right;
	}

	if (nEvents == 60) {
		straight->init(targetTask);
		targetTask = straight;
	}
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs) {

	eventNewMotorAction();
	TaskResult tr;
	return tr;
}