#ifndef AGENT_H
#define AGENT_H

#include "logger.h"
#include <vector>
#include <cassert>
#include <math.h>
#include <memory>
#include <string>

using namespace std;

extern int nEvents;

const float L = 0.147;
const float r = 0.033;

////////////////////////////////// Observations ///////////////////////////////////

// wrapper
struct Point {
	float x = 0.0;
	float y = 0.0;
};

class Observation {
public:

	/*
	* enum for type of observation
	*/
	enum class ObsType {invalid, obstacle, target};

	/* 
	* default constructor 
	* (default obs type == invalid)
	*/
	Observation() {}

	/*
	* initialises lidar observation with location and 
	* observation type (default == obstacle)
	*/
	Observation(float x, float y, ObsType t = ObsType::obstacle) {
		setObservation(x, y, t);
	}

	/*
	* sets location and type of obstacle 
	* (default == obstacle)
	*/
	void setObservation(float x, float y, ObsType t = ObsType::obstacle) {
		location.x = x;
		location.y = y; 
		label = t;
	}

	/*
	* getter for observation location
	* @returns location (struct Point)
	*/
	Point getLocation() const {
		assert(isValid());
		return location;
	}

	/* 
	* getter for distance to another observation
	* @returns euclidian distance to other observation 
	*/
	float getDistance(Observation const &ob) const {
		assert(isValid() && ob.isValid());
		float a = pow(location.x - ob.getLocation().x, 2); 
		float b = pow(location.y - ob.getLocation().y, 2);
		return sqrt(a + b);
	}

	/*
	* getter for radial distance to observation
	* @returns distance from origin to observation
	*/
	float getDistance() const {
		assert(isValid());
		float a = pow(location.x, 2); 
		float b = pow(location.y, 2);
		return sqrt(a + b);
	}

	/*
	* getter for angle from heading to observation
	* @returns angle in rads (pos left, neg right)
	*/
	float getAngle() const {
		assert(isValid());
		return -atan2(location.y, location.x);
	}

	/*
	* getter for observation type
	* @returns observation label 
	*/
	ObsType getLabel() const {
		return label;
	}

	/*
	* invalidates observation when avoided
	*/
	void invalidate() {
		assert(isValid());
		label = ObsType::invalid;
	}

	/*
	* checks whether observation is invalid
	* @returns true if valid, false otherwise
	*/
	bool isValid() const {
		return ObsType::invalid != label;
	}

private:
	Point location;
	ObsType label = ObsType::invalid;
};

////////////////////////////////// Tasks ///////////////////////////////////////

struct ActionInterface {

	/*
	* callback interface for task controls
	* interface implemented in main thread
	*/
	virtual void executeMotorAction(float speedLeft, float speedRight) = 0;
};

class AbstractTask {
public:

	/* 
	* enum for task success/failure criteria
	*/
	enum ResultCodes { nothing = 0, disturbance_gone = 1, 
	failed = 2, new_disturbance = 3 };

	/*
	* task result helper struct
	*/
	struct TaskResult {
		ResultCodes result = nothing;
		Observation newDisturbance;
		void setDisturbance(Observation d) {
			newDisturbance = d;
			result = new_disturbance;
		}
	};

	/*
	* initialises with previous task variables for chaining
	* called by straight tasks, overriden by avoid tasks
	*/
	virtual void init(shared_ptr<AbstractTask> &t, Observation d) {
		// fixme: review variables
		takeAction = t->takeAction;
		desiredMotorDrive = t->desiredMotorDrive;
		motorDriveLeft = t->desiredMotorDrive;
		motorDriveRight = t->desiredMotorDrive;
		disturbance = d;
	}

	/*
	* pure virtual method for task execution to be 
	* implemented by the child class
	*/
	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs) = 0;

	/*
	* registers actuator interface with target task
	* (otherwise pointer passed in task init method)
	*/
	void registerInterface(ActionInterface* ta) {
		takeAction = ta;
	}

	/*
	* sets the initial/desired motor speed
	*/
	void setInitialSpeed(const float speed) {
		// fixme: review variables
		desiredMotorDrive = speed;
		motorDriveLeft = desiredMotorDrive;
		motorDriveRight = desiredMotorDrive;
	}

	/*
	* getter for motor linear velocity
	* @returns kinematic linear velocity
	*/
	float getMotorLinearVelocity() {
		// average of both motors converted to m/s
		return (motorDriveLeft+motorDriveRight) / 2.0 * robotDrive2realSpeed;
	}

	/*
	* getter for motor angular velocity
	* @returns kinematic angular velocity
	*/
	float getMotorAngularVelocity() {
		// fixme: doesn't seems to be quite right
		return (motorDriveLeft - motorDriveRight) / L * robotDrive2realSpeed;
	}

protected:

	/*
	* event handler for motors
	*/
	void eventNewMotorAction() {
		if (nullptr != takeAction) {
			takeAction->executeMotorAction(motorDriveLeft, motorDriveRight);
		}
	}

	// inhereted for chaining
	ActionInterface* takeAction = nullptr;
	float motorDriveLeft = 0.0;
	float motorDriveRight = 0.0;
	float desiredMotorDrive = 0.0;

	// the disturbance
	Observation disturbance;

	// based on lidar rpm
	float taskDuration = 0.0;

	// fixme: review variables
	float robotDrive2realSpeed = 0.1;
	vector<float> actualVelocity = {0, 0};
};

struct StraightTask : AbstractTask {

	float safeZone = 0.2;
	float robotSteering = 0;
	float minDrive = 0.2;
	float maxDrive = 0.75;
	float robotDrive = maxDrive;
	float accSteeringError = 0.0;
	float accSpeedError = 0.0;

	vector<Observation> tp;

	/*
	* task execution step called by agent event handler
	* handles steering error then disturbance detection
	*/
	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs);

	/*
	* finds nearest observation in safe zone around robot
	* safe zone is defined to allow adequate turning 
	* @returns the nearest observation in safe zone 
	*/
	Observation checkSafeZone(const vector<Observation>& obs) {
		Observation disturbance;
		float minx = safeZone;

		for (unsigned i = 0; i < obs.size(); i++) {
			if ((obs[i].isValid())) {
				Point xy = obs[i].getLocation();
				if ((abs(xy.x) < safeZone) && (abs(xy.y) < safeZone) 
					&& (xy.x > 0.15) && (xy.x < minx)) {
					disturbance = obs[i];
					minx = xy.x;
				}
			} 
		}
		return disturbance;
	}

	/*
	* estimates signed mean velocity between 
	* current and previous lidar scans
	* @returns mean linear and lateral velocty
	*/
	vector<float> estimateTrackingVelocity(float samplingrate, 
		const vector<Observation>& obs) {
		vector<Point> current;
		vector<Point> previous;

		for(unsigned i = 0; i < obs.size();i++) {
			if (obs[i].isValid()) {
				if (i < tp.size() ) {
					if (tp[i].isValid()) {
						current.push_back(obs[i].getLocation());
						previous.push_back(tp[i].getLocation());
					}
				}
			}
		}

		float dx = 0.0;
		float dy = 0.0;

		vector<float> vxy = {0.0, 0.0};
		for (unsigned i = 0; i < current.size(); i++) {
			dx = current[i].x - previous[i].x;
			dy = current[i].y - previous[i].y;

			vxy[0] += dx / (1/samplingrate);
			vxy[1] += dy / (1/samplingrate);
		} 

		vxy[0] = vxy[0] / current.size();
		vxy[1] = vxy[1] / current.size();

		float min = minDrive * robotDrive2realSpeed;
		float max = maxDrive * robotDrive2realSpeed;

		// fixme: crude error handling 
		if (abs(vxy[1]) > max || abs(vxy[1]) > abs(vxy[0])) {
			vxy = {0, 0};
		}

		return vxy;
	}
};

template <int signedYawScalar>
struct Rotate90Task : AbstractTask {

	// overrides init method in base class
	virtual void init(shared_ptr<AbstractTask> &t, Observation d) {
		AbstractTask::init(t, d);
		motorDriveLeft = desiredMotorDrive *(float)signedYawScalar;
		motorDriveRight = -desiredMotorDrive *(float)signedYawScalar;
	}

	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs) {
		TaskResult tr;

		// distrubance angle
		// same movement as turning
		// 90 degrees from theta 0

		float startAngle = 0.0; 
		if (disturbance.isValid()) {
			startAngle = disturbance.getAngle();
		}

		float omega = getMotorAngularVelocity();
		float angle = startAngle + omega * taskDuration * 2;
		logger.printf("duration = %f startAngle = %f  angle = %f  omega = %f \n", taskDuration, startAngle, angle, omega);

		if (abs(angle - startAngle) > M_PI/2) {
			tr.result = ResultCodes::disturbance_gone;
			logger.printf("Cleared !! angle = %f\n", abs(angle - startAngle));
			return tr;
		}

		eventNewMotorAction();
		taskDuration += (1/samplingrate);
		return tr;
	};
};

struct Rotate90Left : Rotate90Task<1> {};
struct Rotate90Right : Rotate90Task<-1> {};


////////////////////////////// Agent ////////////////////////////////////////////////

class Agent {
public:

	void eventNewRelativeCoordinates(float samplingrate,
		const vector<Observation>& obs);

	void setTargetTask(shared_ptr<AbstractTask> t) {
		targetTask = t;
		currentTask = t;
	}

private:

	// default task straight
	shared_ptr<AbstractTask> targetTask;

	// the task we are in
	shared_ptr<AbstractTask> currentTask;

	// the plan if available
	vector<shared_ptr<AbstractTask>> plan;

	void saveMap(const vector<Observation>& obs) {
		char tmp[256];
		sprintf(tmp,"../test/map%03d.dat",nEvents);
		fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		for(unsigned i = 0; i < obs.size();i++) {
			if (obs[i].isValid()) {
				fprintf(f,"%4.4f %4.4f %4.4f %4.4f\n",
				obs[i].getLocation().x,
				obs[i].getLocation().y,
				obs[i].getDistance(),
				obs[i].getAngle());
			}
		}
		fclose(f);
	}
};


#endif