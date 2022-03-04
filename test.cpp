#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (action == 1) {
			turnLeft(&alphabot, 0.5);
		} else if (action == 0) {
			forward(&alphabot, 0.5);
		} else if (action == 2) {
			turnRight(&alphabot, 0.5);
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
		alphabot->setLeftWheelSpeed(speed - 0.2);
		alphabot->setRightWheelSpeed(speed + 0.2);
	}

	void turnRight(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed + 0.2);
		alphabot->setRightWheelSpeed(speed - 0.2);
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
