#include "agent.h"

int nEvents = 0;
float detectionThreshold = 0;

Logger logger;

bool jobdone = false;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	AbstractTask::TaskResult tr = currentTask->taskExecutionStep(samplingrate, obs);

	bool react = false;
	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			if ((obs[i].getDistance() < reactionThreshold) 
				&& (obs[i].getDistance() > lidarMinRange)
				&& abs(obs[i].getLocation().y) < wheelbase) {
				react = true;
			}
		} 
	}

	if (tr.result == 3) {
		
		if (!jobdone) {
			planner->eventNewDisturbance(plan, obs, tr.newDisturbance);
			jobdone = true;
		}
	
		float angle = tr.newDisturbance.getAngle();
		if ((angle < 0) && react) {
			auto left = make_shared<Rotate90Left>();
			left->init(currentTask, tr.newDisturbance);
			currentTask = left;
		} else if (react) {
			auto right = make_shared<Rotate90Right>();
			right->init(currentTask, tr.newDisturbance);
			currentTask = right;
		}
	}

	if (tr.result == 1) {
		jobdone = false;
		currentTask = targetTask;
	}

	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs) {

	/* step 1: adjust for steering error */
	vector<Point> current;
	vector<Point> previous;

	for(unsigned i = 0; i < obs.size();i++) {
		if (obs[i].isValid()) {
			if (i < prevTrackingObs.size() ) {
				if (prevTrackingObs[i].isValid()) {
					current.push_back(obs[i].getLocation());
					previous.push_back(prevTrackingObs[i].getLocation());
				}
			}
		}
	}

	float dx = 0;
	float dy = 0;

	// todo: could try bining the v
	vector<float> trackingVelocity = {0, 0};
	for (unsigned i = 0; i < current.size(); i++) {
		dx = current[i].x - previous[i].x;
		dy = current[i].y - previous[i].y;

		trackingVelocity[0] += dx / (1/samplingrate);
		trackingVelocity[1] += dy / (1/samplingrate);
	} 

	trackingVelocity[0] = trackingVelocity[0] / current.size();
	trackingVelocity[1] = trackingVelocity[1] / current.size();
	float motorLinearVelocity = getMotorLinearVelocity();

	// fixme: crude error handling 
	if (abs(trackingVelocity[1]) > motorLinearVelocity || 
		abs(trackingVelocity[1]) > abs(trackingVelocity[0])) {
		trackingVelocity[1] = 0;
	}

	prevTrackingObs = obs;

	float steeringError = 0;
	if (abs(trackingVelocity[1]) > 0) {
		// setpoint is zero but need the sign
		steeringError = trackingVelocity[1];	
		accSteeringError += steeringError;

		// fixme: crude tracking cutoff
		if (abs(accSteeringError) > 0.07) {
			accSteeringError = 0;
		}
	} 

	// fixme: tune gain for PI steering control
	robotSteering = steeringError * 1.0 + accSteeringError * 1.0; 
	motorDriveLeft = desiredMotorDrive + robotSteering;
	motorDriveRight = desiredMotorDrive - robotSteering;
	eventNewMotorAction();

	/* step 2: look for disturbance  */
	// fixme: need to adjust linear velocity with error
	detectionThreshold = motorLinearVelocity * disturbanceLookahead;

	Observation disturbance;
	float minx = detectionThreshold;

	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			if ((abs(location.x) < detectionThreshold) 
				&& (abs(location.y) <= reactionThreshold) 
				&& (location.x > lidarMinRange) 
				&& (location.x < minx)) {
				disturbance = obs[i];
				minx = location.x;
			}
		} 
	}

	TaskResult tr;
	if (disturbance.isValid()) {
		accSteeringError = 0;
		tr.setDisturbance(disturbance);
	} 

	taskDuration += (1/samplingrate);
	return tr;
}

vector<shared_ptr<AbstractTask>> SimpleInvariantLTL::eventNewDisturbance(vector<shared_ptr<AbstractTask>> plan, 
	const vector<Observation>& obs, const Observation& _disturbance) {
 	
 	/* step 1: decide which state should be accepting */

	// forward simulate disturbance
 	float d = _disturbance.getLocation().x - reactionThreshold;
 	Observation disturbance(disturbance.getLocation().x - d, disturbance.getLocation().y);

 	// forward simulate observations
 	vector<Observation> forwardSimObs;
 	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - d, location.y);
			forwardSimObs.push_back(ob);
		} 
	}

	// decide which subtree
	vector<Observation> subtree;
	for (unsigned i = 0; i < obs.size(); i++) {
		if (forwardSimObs[i].isValid()) {
			if (disturbance.getAngle() < 0) {
				subtree.push_back(forwardSimObs[i]);
			} 
			if (disturbance.getAngle() > 0) {
				subtree.push_back(forwardSimObs[i]);
			}
		}	
	}

	if (disturbance.getAngle() < 0) {
		logger.printf("left subtree...\n");
	} else {
		logger.printf("right subtree...\n");
	}

	// nearest ob in subtree
	Observation nearest;
	float miny = detectionThreshold;
	for (unsigned i = 0; i < subtree.size(); i++) {
		if (subtree[i].isValid()) {
			Point location = subtree[i].getLocation();
			if ((abs(location.y) < detectionThreshold) 
				&& (abs(location.y) <= reactionThreshold) 
				&& (abs(location.y) > lidarMinRange) 
				&& (abs(location.y) < miny)) {
				nearest = subtree[i];
				miny = abs(location.y);
			}
		} 
	}
	
	if (nearest.isValid()) {
		logger.printf("nearest x = %f y = %f dist = %f angle = %f\n", 
			nearest.getLocation().x, nearest.getLocation().y, 
			nearest.getDistance(), nearest.getAngle());
	}

 	/* step 2: generate path */

 	/* step 3: return sequence of tasks */
 	return plan;
}