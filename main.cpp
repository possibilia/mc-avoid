#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {

	}

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turn(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
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
	alphabot.start();

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	unsigned f = 0;
	unsigned t = 0;

	while(true) {
		unsigned action = data.getAction();
		if ((action == 1) & (t == 0)) {
			turn(&alphabot, 0.2);
			f = 0;
			t = 1;
		} else if ((action == 0) & (f == 0)) {
			forward(&alphabot, 0.5);
			f = 1;
			t = 0;
		}
	}

	alphabot.stop();
	lidar.stop();
	return 0;
}