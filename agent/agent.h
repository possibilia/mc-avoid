#ifndef AGENT_H
#define AGENT_H

#include "logger.h"
#include <vector>
#include <cassert>
#include <math.h>
#include <memory>

using namespace std;

extern int nEvents;

////////////////////////////////// Observations ///////////////////////////////////

struct Point {
	float x = 0.0;
	float y = 0.0;
};

class Observation {

public:
	enum class ObsType {invalid, obstacle, target};

	Observation() {}

	Observation(float x, float y, ObsType t = ObsType::obstacle) {
		setObservation(x, y, t);
	}

	void setObservation(float x, float y, ObsType t = ObsType::obstacle) {
		location.x = x;
		location.y = y; 
		label = t;
	}

	Point getLocation() const {
		assert(isValid());
		return location;
	}

	// distance between two observations
	float getDistance(Observation const &ob) const {
		assert(isValid() && ob.isValid());
		float a = pow(location.x - ob.getLocation().x, 2); 
		float b = pow(location.y - ob.getLocation().y, 2);
		return sqrt(a + b);
	}

	// distance from origin
	float getDistance() const {
		assert(isValid());
		float a = pow(location.x, 2); 
		float b = pow(location.y, 2);
		return sqrt(a + b);
	}

	float getAngle() const {
		assert(isValid());
		return -atan2(location.y, location.x);
	}

	ObsType getLabel() const {
		return label;
	}

	void invalidate() {
		assert(isValid());
		label = ObsType::invalid;
	}

	bool isValid() const {
		return ObsType::invalid != label;
	}

private:
	Point location;
	ObsType label = ObsType::invalid;
};

////////////////////////////////// Tasks ////////////////////////////////////////

struct ActionInterface {
	virtual void executeMotorAction(float speedLeft, float speedRight) = 0;
};

class AbstractTask {
public:
	enum ResultCodes { nothing = 0, disturbance_gone = 1, failed = 2, new_disturbance = 3 };

	struct TaskResult {
		ResultCodes result = nothing;
		Observation newDisturbance;
		void setDisturbance(Observation d) {
			newDisturbance = d;
			result = new_disturbance;
		}
	};

	virtual TaskResult robotExecutionStep(float samplingrate, 
		const vector<Observation>& obs) = 0;

	void registerInterface(ActionInterface* ta) {
		takeAction = ta;
	}

protected:
	void eventNewMotorAction() {
		if (nullptr != takeAction) {
			takeAction->executeMotorAction(motorDriveLeft,motorDriveRight);
		}
	}

	ActionInterface* takeAction = nullptr;
	float motorDriveLeft = 0.2;
	float motorDriveRight = 0.2;
	float robotDrive2realSpeed = 0.1;
	Observation disturbance;
};

struct StraightTask : AbstractTask {
	virtual TaskResult robotExecutionStep(float samplingrate, 
		const vector<Observation>& obs);
};


////////////////////////////// Agent ////////////////////////////////////////////////


class Agent {
public:
	void eventNewRelativeCoordinates(float samplingrate,
		const vector<Observation>& obs);

	void setTargetTask(shared_ptr<AbstractTask> t) {
		targetTask = t;
	}

private:

	vector<Observation> previousTrackingPoints;

	shared_ptr<AbstractTask> targetTask;

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