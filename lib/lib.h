#ifndef control_h
#define control_h

#include "logger.h"
#include <vector>
#include <cassert>
#include <math.h>
#include <memory>

using namespace std;

extern int nEvents;

////////////////////////////////// Object handling ////////////////////////////

struct Point {
	float x = 0.0;
	float y = 0.0;
};

// generalised structure
class ObjectIdentifier {

public:
	enum class ObjectType {invalid, obstacle, target};

	ObjectIdentifier() {}

	ObjectIdentifier(float x, float y, ObjectType t = ObjectType::obstacle) {
		setObject(x, y, t);
	}

	void setObject(float x, float y, ObjectType t = ObjectType::obstacle) {
		location.x = x;
		location.y = y; 
		label = t;
	}

	Point getLocation() const {
		assert(isValid());
		return location;
	}

	// distance between two objects
	float getDistance(ObjectIdentifier const &object) const {
		assert(isValid() && object.isValid());
		float a = pow(location.x - object.getLocation().x, 2); 
		float b = pow(location.y - object.getLocation().y, 2);
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

	ObjectType getLabel() const {
		return label;
	}

	void invalidate() {
		assert(isValid());
		label = ObjectType::invalid;
	}

	bool isValid() const {
		return ObjectType::invalid != label;
	}

private:
	Point location;
	ObjectType label = ObjectType::invalid;
};

////////////////////////////////// Task hierarchy ////////////////////////////

// interface for actuators
struct TakeAction {
	virtual void motorAction(float speedLeft, float speedRight) = 0;
};

class AbstractTask {
public:
	enum ResultCodes { nothing = 0, 
	disturbance_gone = 1, failed = 2, new_disturbance = 3 };

	struct TaskResult {
		ResultCodes result = nothing;
		ObjectIdentifier newDisturbance;
		void setDisturbance(ObjectIdentifier d) {
			newDisturbance = d;
			result = new_disturbance;
		}
	};

	virtual TaskResult robotExecutionStep(float samplingrate, 
		const vector<ObjectIdentifier>& objects) = 0;

	void registerTakeAction(TakeAction* ta) {
		takeAction = ta;
	}

protected:
	void newMotorAction() {
		if (nullptr != takeAction) {
			takeAction->motorAction(motorDriveLeft,motorDriveRight);
		}
	}

	TakeAction* takeAction = nullptr;
	float motorDriveLeft = 0.2;
	float motorDriveRight = 0.2;
	float robotDrive2realSpeed = 0.1;
	ObjectIdentifier disturbance;
};

struct StraightTask : AbstractTask {
	virtual TaskResult robotExecutionStep(float samplingrate, 
		const vector<ObjectIdentifier>& objects);
};


////////////////////////////// Event handler //////////////////////////////


class Actor {
public:
	void eventNewRelativeCoordinates(float samplingrate,
		const vector<ObjectIdentifier>& objects);

	void setTargetTask(shared_ptr<AbstractTask> t) {
		targetTask = t;
	}

private:

	vector<ObjectIdentifier> previousTrackingPoints;

	shared_ptr<AbstractTask> targetTask;

	void saveMap(const vector<ObjectIdentifier>& objects) {
		char tmp[256];
		sprintf(tmp,"../test/map%03d.dat",nEvents);
		fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		for(unsigned i = 0; i < objects.size();i++) {
			if (objects[i].isValid()) {
				fprintf(f,"%4.4f %4.4f %4.4f %4.4f\n",
				objects[i].getLocation().x,
				objects[i].getLocation().y,
				objects[i].getDistance(),
				objects[i].getAngle());
			}
		}
		fclose(f);
	}
};


#endif