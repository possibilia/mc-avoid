#include "alphabot.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		stop(&alphabot, speed);
		turn(&alphabot, speed);

		if (speed > 1.0) {
			speed = 0.0;
		} else {
			speed += 0.1;
		}
	}

	void setAction(int action) {

	}

private:
	float speed = 0.0;

	void stop(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turn(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(-speed);
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
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	while(true) {
		std::string command;
		std::cin >> command;

		if (command == "q") {
			alphabot.stop();
			break;
		}
	}

	return 0;
}
