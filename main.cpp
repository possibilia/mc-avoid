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

	std::cout << "Hello" << "\n";
	while(true) {
		std::string command;
		std::cin >> command;

		if (command == "q") {
			break;
		}

		// unsigned action = data.getAction();
		// std::cout << action << std::endl;
		// if (action == 1) {
		// 	std::cout << "1" << std::endl;
		// 	turn(&alphabot, 0.2);
		// } else {
		// 	std::cout << "0" << std::endl;
		// 	forward(&alphabot, 0.2);
		// }
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
