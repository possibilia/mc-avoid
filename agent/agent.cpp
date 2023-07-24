#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) {

	shared_ptr<AbstractTask> rigth = make_shared<Rotate90Right>();
	shared_ptr<AbstractTask> straight = make_shared<StraightTask>();

	AbstractTask::TaskResult tr = targetTask->taskExecutionStep(samplingrate, obs);

	//saveMap(obs);
	nEvents++;

	if (nEvents == 20) {
		rigth->init(targetTask);
		targetTask = rigth;
	}

	if (tr.result == 1) {
		straight->init(targetTask);
		targetTask = straight;
	}
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs) {
	TaskResult tr;

	progress += getMotorLinearVelocity() * (1/samplingrate);
	logger.printf("meters = %f\n", progress);

	eventNewMotorAction();
	return tr;
}