#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (action == 1) {
			turnLeft(&alphabot, 0.3);
		} else if (action == 0) {
			forward(&alphabot, 0.3);
		} else if (action == 2) {
			turnRight(&alphabot, 0.3);
		}
	}

	void setAction(unsigned action) {
		this->action = action;
	}

private:
	unsigned action = 0;

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turnLeft(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turnRight(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(speed);
	}

};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		if ((action == 1) || (action == 2)) {
			action = 0;
		}

		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi > 0.0) & (data.phi < 1.0)) {
				action = 2;
			} else if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {
				action = 1;
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
	ControlCallback control;

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
