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
			plan = planner->eventNewDisturbance(plan, obs, tr.newDisturbance);

			if (!plan.empty()) {
				jobdone = true;
			} 
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
				&& (abs(location.y) <= wheelbase) 
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
	const vector<Observation>& obs, const Observation& disturbance) {

	/* step 1: decide which states should be accepting */

	// forward simulate disturbance
 	float d = abs(disturbance.getLocation().x - reactionThreshold);
 	Observation forwardSimDisturbance(disturbance.getLocation().x - d, disturbance.getLocation().y);

 	// logger.printf("orig disturbance x = %f y = %f\n", disturbance.getLocation().x, disturbance.getLocation().y);
 	// logger.printf("sim disturbance x = %f y = %f\n", forwardSimDisturbance.getLocation().x, forwardSimDisturbance.getLocation().y);

 	// forward simulate flanks
 	// S_flanks = {s1, s2, s3, s4}
 	vector<Observation> flanks;
 	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - d, location.y);
			// filter out the flanks
			if ((abs(ob.getLocation().y) < detectionThreshold) 
				&& (abs(ob.getLocation().x) <= reactionThreshold) 
				&& (abs(ob.getLocation().y) > reactionThreshold)) {
				flanks.push_back(ob);
			}
		} 
	}

	// logger.printf("sim size = %d reactionThreshold = %f  detectionThreshold = %f\n", 
	// 	flanks.size(), reactionThreshold, detectionThreshold);

	// S_left = {s1, s3}
	vector<Observation> leftFlank;
	// S_right = {s2, s4}
	vector<Observation> rightFlank;
	for (unsigned i = 0; i < flanks.size(); i++) {
		if (flanks[i].getLocation().y > 0) {
			leftFlank.push_back(flanks[i]);
		} else {
			rightFlank.push_back(flanks[i]);
		}
	}

	shared_ptr<AbstractTask> t = make_shared<StraightTask>();
 	plan.push_back(t);

	// way is clear either direction S_flanks = {s1, s2, s3, s4}
	if ((leftFlank.size() == 0)  && (rightFlank.size() == 0)) {
		// two accepting states
		logger.printf("left/right -> straight\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	// disturbance left so go right S_right = {s2, s4}
	if ((leftFlank.size() > 0) && (rightFlank.size() == 0)) {
		logger.printf("right -> straight\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	// disturbance right so go left S_left = {s1, s3}
	if ((leftFlank.size() == 0)  && (rightFlank.size() > 0)) {
		logger.printf("left -> straight\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	// nearest disturbance left
	Observation nearestLeftFlank;
	float miny = detectionThreshold;
	for (unsigned i = 0; i < leftFlank.size(); i++) {
		if (leftFlank[i].isValid()) {
			Point location = leftFlank[i].getLocation();
			if ((abs(location.y) < miny)
				&& (abs(location.x) < reactionThreshold)) {
				nearestLeftFlank = leftFlank[i];
				miny = abs(location.y);
			}
		} 
	}

	// nearest disturbance right
	Observation nearestRightFlank;
	miny = detectionThreshold;
	for (unsigned i = 0; i < rightFlank.size(); i++) {
		if (rightFlank[i].isValid()) {
			Point location = rightFlank[i].getLocation();
			if ((abs(location.y) < miny)
				&& (abs(location.x) < reactionThreshold)) {
				nearestRightFlank = rightFlank[i];
				miny = abs(location.y);
			}
		} 
	}

	// check if the robot has room to plan an extra step
	float leftTranslation = nearestLeftFlank.getLocation().y - reactionThreshold;
	bool leftFlankSafe = leftTranslation > reactionThreshold * 0.1;

	float rigthTranslation = abs(nearestRightFlank.getLocation().y) - reactionThreshold;
	bool rightFlankSafe = rigthTranslation > reactionThreshold * 0.1;

	// no flank safe so turn around S_back = {s13, s14}
	if ((leftFlank.size() > 0)  && (rightFlank.size() > 0) 
		&& !(leftFlankSafe || rightFlankSafe)) {
		logger.printf("2left/2right -> straight\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	// S_NE = {s6, s8}
	vector<Observation> northEastHorizon;
	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - d, location.y);
			// north east horizon states
			if ((ob.getLocation().x > detectionThreshold) 
				&& (ob.getLocation().x <= reactionThreshold * 1.5) 
				&& (abs(ob.getLocation().y) > reactionThreshold)) {
				northEastHorizon.push_back(ob);
			}

		} 
	}

	// room either way so choose direction at random
	// S_north = {s5, s7, s6, s8), S_south = {s9, s11, s10, s12}
	if ((leftFlank.size() > 0)  && (rightFlank.size() > 0) 
		&& (leftFlankSafe && rightFlankSafe)) {
		// four accepting states
		logger.printf("4 possible options...\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	if ((leftFlank.size() > 0)  && (rightFlank.size() > 0) 
		&& (leftFlankSafe && !rightFlankSafe)) {
		// two accepting states
		logger.printf("2 possible options left...\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}

	if ((leftFlank.size() > 0)  && (rightFlank.size() > 0) 
		&& (!leftFlankSafe && rightFlankSafe)) {
		// two accepting states
		logger.printf("2 possible options right...\n");
		// set relevant states
		// get path 
		// extract tasks
		return plan;
	}


 	/* step 2: generate path */

 	/* step 3: return sequence of tasks */

 	return plan;
}
