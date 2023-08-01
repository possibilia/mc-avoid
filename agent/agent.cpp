#include "agent.h"

int nEvents = 0;
float detectionThreshold = 0;

Logger logger;

bool jobdone = false;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	AbstractTask::TaskResult tr = currentTask->taskExecutionStep(samplingrate, obs);
	logger.printf("Task result %d\n", tr.result);
	logger.printf("Plan size %d\n", plan.size());

	if (tr.result == AbstractTask::disturbance_gone)
	{
		if (plan.empty()) {
			currentTask = targetTask;
		} else {
			plan[0]->init(currentTask, tr.newDisturbance);
			currentTask = plan[0];
			plan.erase(plan.begin());
		}
	}

	if (tr.result == AbstractTask::new_disturbance) {
		if (tr.newDisturbance.isValid() && plan.empty()) {
			plan = planner->eventNewDisturbance(obs, tr.newDisturbance);
		}

		if (tr.newDisturbance.getDistance() <= reactionThreshold) {
			plan[0]->init(currentTask, tr.newDisturbance);
			currentTask = plan[0];
			plan.erase(plan.begin());
		}
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

vector<shared_ptr<AbstractTask>> StateMachineLTL::eventNewDisturbance(
	const vector<Observation>& obs, const Observation& disturbance) {

	set<int> accept;

	// get offset from distrbance for forward simulation 
 	float northOffset = abs(disturbance.getLocation().x - reactionThreshold);

 	// forward simulate obs and filter
 	vector<Observation> westHorizon;
 	vector<Observation> eastHorizon;
 	for (unsigned i = 0; i < obs.size(); i++) {
		if (obs[i].isValid()) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - northOffset, location.y);
			// S_w = {s1, s3}
			if ((ob.getLocation().y < lateralHorizon)
				&& (abs(ob.getLocation().x) <= reactionThreshold) 
				&& (ob.getLocation().y > reactionThreshold)) {
				westHorizon.push_back(ob);
			}
			// S_e = {s2, s4}
			if ((ob.getLocation().y > -lateralHorizon)
				&& (abs(ob.getLocation().x) <= reactionThreshold) 
				&& (ob.getLocation().y < -reactionThreshold)) {
				eastHorizon.push_back(ob);
			}
		} 
	}

	logger.printf("east size = %d  west size = %d\n", eastHorizon.size(), westHorizon.size());

	// way is clear either direction S_w U S_e = {s1, s2, s3, s4}
	if ((westHorizon.size() == 0)  && (eastHorizon.size() == 0)) {
		logger.printf("left/right -> straight\n");
		// set relevant states
		accept.insert(3);
		accept.insert(4);
		vector<shared_ptr<AbstractTask>> plan = generatePlan(accept);
		logger.printf("3 & 4: size = %d\n", plan.size());

		return plan;
	}

	// disturbance left so go right S_e = {s2, s4}
	if ((westHorizon.size() > 0) && (eastHorizon.size() == 0)) {
		logger.printf("right -> straight\n");
		accept.insert(4);
		vector<shared_ptr<AbstractTask>> plan = generatePlan(accept);
		logger.printf("4: size = %d\n", plan.size());
		return plan;
	}

	// disturbance right so go left S_w = {s1, s3}
	if ((westHorizon.size() == 0)  && (eastHorizon.size() > 0)) {
		logger.printf("left -> straight\n");
		accept.insert(3);
		vector<shared_ptr<AbstractTask>> plan = generatePlan(accept);
		logger.printf("3: size = %d\n", plan.size());
		return plan;
	}

	// nearest west
	Observation nearestWest;
	float miny = detectionThreshold;
	for (unsigned i = 0; i < westHorizon.size(); i++) {
		if (westHorizon[i].isValid()) {
			Point location = westHorizon[i].getLocation();
			if ((abs(location.y) < miny)
				&& (abs(location.x) < reactionThreshold)) {
				nearestWest = westHorizon[i];
				miny = abs(location.y);
			}
		} 
	}

	// nearest east
	Observation nearestEast;
	miny = detectionThreshold;
	for (unsigned i = 0; i < eastHorizon.size(); i++) {
		if (eastHorizon[i].isValid()) {
			Point location = eastHorizon[i].getLocation();
			if ((abs(location.y) < miny)
				&& (abs(location.x) < reactionThreshold)) {
				nearestEast = eastHorizon[i];
				miny = abs(location.y);
			}
		} 
	}

	// check if the robot has room to plan an extra step
	float westOffset = nearestWest.getLocation().y - reactionThreshold;
	float eastOffset = nearestEast.getLocation().y + reactionThreshold;

	logger.printf("east y = %f west y = %f\n", nearestEast.getLocation().y, nearestWest.getLocation().y);
	logger.printf("eastOffset = %f westOffset = %f\n", eastOffset, westOffset);

	bool westDirectionSafe = westOffset > 0.02; // max size of object
	bool eastDirectionSafe = abs(eastOffset) > 0.02; // max size of object

	logger.printf("east safe = %d  west safe = %d\n", eastDirectionSafe, westDirectionSafe);

	// no room so go south S_s = {s13, s14}
	if (!(westDirectionSafe || eastDirectionSafe)) {
		logger.printf("2left/2right -> straight\n");
		accept.insert(14);
		vector<shared_ptr<AbstractTask>> plan = generatePlan(accept);
		logger.printf("14: size = %d", plan.size());
		return plan;
	}

	// forward sim and east offset
	vector<Observation> northEastHorizon;
	vector<Observation> southEastHorizon;
	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - northOffset, location.y - eastOffset);
			// S_ne = {s6, s8}
			if ((ob.getLocation().x > reactionThreshold) 
				&& (ob.getLocation().x <= reactionThreshold * 1.5) 
				&& (abs(ob.getLocation().y) <= wheelbase)) {
				northEastHorizon.push_back(ob);
			}
			// S_se = {s10, s12}
			if ((ob.getLocation().x < -reactionThreshold) 
				&& (ob.getLocation().x >= -reactionThreshold * 1.5) 
				&& (abs(ob.getLocation().y) <= wheelbase)) {
				southEastHorizon.push_back(ob);
			}
		} 
	}

	// forward sim and west offset
	vector<Observation> northWestHorizon;
	vector<Observation> southWestHorizon;
	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			Point location = obs[i].getLocation();
			Observation ob(location.x - northOffset, location.y - westOffset);
			// S_nw = {s5, s7}
			if ((ob.getLocation().x > reactionThreshold) 
				&& (ob.getLocation().x <= reactionThreshold * 1.5) 
				&& (abs(ob.getLocation().y) <= wheelbase)) {
				northWestHorizon.push_back(ob);
			}
			// S_sw = {s9, s11}
			if ((ob.getLocation().x < -reactionThreshold) 
				&& (ob.getLocation().x >= -(reactionThreshold * 1.5)) 
				&& (abs(ob.getLocation().y) <= wheelbase)) {
				southWestHorizon.push_back(ob);
			}
		} 
	}

	logger.printf("north east size = %d south east size = %d\n", northEastHorizon.size(), southEastHorizon.size());
	logger.printf("north west size = %d south west size = %d\n", northWestHorizon.size(), southWestHorizon.size());

	// east north/south horizon states
	if ((westDirectionSafe && eastDirectionSafe) 
		|| (!westDirectionSafe && eastDirectionSafe)) {
		// S_ne = {s6, s8}
		if (northEastHorizon.size() == 0) {
			logger.printf("north east safe!\n");
			// set state
			accept.insert(8);
		}
		// S_se = {s10, s12}
		if (southEastHorizon.size() == 0) {
			logger.printf("south east safe!\n");
			// set state
			accept.insert(12);
		}
	}

	// west north/south horizon states
	if ((westDirectionSafe && eastDirectionSafe) 
		|| (westDirectionSafe && !eastDirectionSafe)) {
		// S_nw = {s5, s7}
		if (northWestHorizon.size() == 0) {
			logger.printf("north west safe!\n");
			// set state
			accept.insert(7);
		}
		// S_sw = {s9, s11}
		if (southWestHorizon.size() == 0) {
			logger.printf("south west safe!\n");
			// set state
			accept.insert(11);
		}
	}

	
	vector<shared_ptr<AbstractTask>> plan = generatePlan(accept);
	logger.printf("accept size = %d size = %d\n", accept.size(), plan.size());

 	return plan;
}
