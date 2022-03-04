#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>

class DifferentialDriveCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		unsigned f = 0;
		unsigned t = 0;
		unsigned s = 0;

		if ((action == 1) & (t == 0)) {
			turn(&alphabot, 0.2);
			f = 0;
			t = 1;
			s = 0;
		} else if ((action == 0) & (f == 0)) {
			forward(&alphabot, 0.5);
			f = 1;
			t = 0;
			s = 0;
		} else if ((action == 2) & (s == 0)) {
			forward(&alphabot, 0.3);
			f = 0;
			t = 0;
			s = 1;
		}
	}

	void setAction(unsigned action) {
		this->action = action;
	}

private:
	const float wheelBase = 0.00335;
	const float wheelRadius = 0.142;

	unsigned action = 0;

	float * forwardKinematics(float wL, float wR) {
		float velocities[2];

		float v = (wheelRadius / 2) * (wR + wL);
		float w = (wheelRadius / wheelBase) * (wR - wL);

		velocities[0] = v;
		velocities[1] = w;

		return velocities;
	}

	float * inverseKinematics(float vref, float wref) {
		float rpms[2];

		float wL = (1 / wheelRadius) * (vref - ((wref * wheelBase) / 2));
		float wR = (1 / wheelRadius) * (vref + ((wref * wheelBase) / 2));

		rpms[0] = wL;
		rpms[1] = wR;

		return rpms;
	}
};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		if (action == 1) {
			action = 0;
		}

		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 0.2) & (data.r >= 0.0) & 
				(data.phi < 2.0) & (data.phi > -2.0)) {
				action = 1;
			} else if ((data.valid) & (data.r >= 0.2) & (data.r < 0.4) & 
				(data.phi < 2.0) & (data.phi > -2.0)) {
				action = 2;
			}
		}
	}

	unsigned getAction() {	
		return action;
	}

private:
	unsigned action = 0;

};

int main(int, char **) { 
	DataInterface data;
	DifferentialDriveCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	while(true) {
		unsigned action;
		action = data.getAction();
		control.setAction(action);
	}

	alphabot.stop();
	lidar.stop();

	return 0;
}
