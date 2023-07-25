#include "agent.h"

int nEvents = 0;

Logger logger;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	setMeanLateralError(obs);
	float meanLateralError = getMeanLateralError();
	//logger.printf("tracking error : %f \n", trackingError);

	AbstractTask::TaskResult tr = targetTask->taskExecutionStep(samplingrate, obs, meanLateralError);
 
	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs, float meanLateralError) {

	TaskResult tr;

	Observation disturbance = checkSafeZone(obs);
	if (disturbance.isValid()) {
		tr.setDisturbance(disturbance);
		logger.printf(" distrubance r = %f, theta = %f \n", 
			disturbance.getDistance(), disturbance.getAngle());
	} else {
		logger.printf(" distrubance r = %f, theta = %f \n", 0.0, 0.0);
	}

	float distance = getMotorLinearVelocity() * taskDuration;
	//logger.printf("meters = %f\n", distance);

	eventNewMotorAction(meanLateralError);
	taskDuration += (1/samplingrate);
	return tr;
}