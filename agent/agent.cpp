#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	setMeanLateralError(obs);
	float meanLateralError = getMeanLateralError();
	//logger.printf("tracking error : %f \n", trackingError);

	AbstractTask::TaskResult tr = currentTask->taskExecutionStep(samplingrate, obs, meanLateralError);

	if (tr.result == 3) {
		float angle = tr.newDisturbance.getAngle();
		if (angle < 0.0) {
			auto left = make_shared<Rotate90Left>();
			left->init(currentTask, tr.newDisturbance);
			currentTask = left;
		} else {
			auto right = make_shared<Rotate90Right>();
			right->init(currentTask, tr.newDisturbance);
			currentTask = right;
		}
	}

	if (tr.result == 1) {
		currentTask = targetTask;
	}

	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs, float meanLateralError) {

	TaskResult tr;

	Observation disturbance = checkSafeZone(obs);
	if (disturbance.isValid()) {
		tr.setDisturbance(disturbance);
		// logger.printf(" result = %d, r = %f, theta = %f \n", tr.result,
		// 	disturbance.getDistance(), disturbance.getAngle());
	} else {
		//logger.printf(" result = %d, r = %f, theta = %f \n", tr.result, 0.0, 0.0);
	}

	eventNewMotorAction(meanLateralError);
	taskDuration += (1/samplingrate);
	return tr;
}