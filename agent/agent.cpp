#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	setTrackingError(obs);
	float trackingError = getTrackingError();
	logger.printf("tracking error : %f \n", trackingError);

	AbstractTask::TaskResult tr = targetTask->taskExecutionStep(samplingrate, obs, trackingError);
	shared_ptr<AbstractTask> right  = make_shared<Rotate90Right>();
	shared_ptr<AbstractTask> straight  = make_shared<StraightTask>();

	if (nEvents == 20) {
		right->init(targetTask);
		targetTask = right;
	}

	if (tr.result == AbstractTask::ResultCodes::disturbance_gone) {
		straight->init(targetTask);
		targetTask = straight;
	}
 
	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs, float trackingError) {
	TaskResult tr;

	float distance = getMotorLinearVelocity() * taskDuration;
	//logger.printf("meters = %f\n", distance);

	eventNewMotorAction(trackingError);
	taskDuration += (1/samplingrate);
	return tr;
}