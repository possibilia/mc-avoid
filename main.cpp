#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>

void forward(AlphaBot* alphabot, float speed);

void turn(AlphaBot* alphabot, float speed);

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 0.2) & (data.r > 0.0)) {
				action = 1;
			} else {
				action = 0;
			}
		}
	}

	int getAction() {
		return action;
	}

private:
	unsigned action = 0;

};

int main(int, char **) {
	AlphaBot alphabot;
	alphabot.start();

	DataInterface data;

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	while(true) {
		std::string command;
		std::cin >> command;

		if (command == "q") {
			break;
		}

		int action = data.getAction();
		if (action == 1) {
			turn(&alphabot, 0.2);
		} else {
			forward(&alphabot, 0.2);
		}
	}

	// stop threads
	alphabot.stop();
	lidar.stop();

	// exit 
	return 0;
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
