#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <curl/curl.h>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		std::cout << "Left: " << leftDistance << "\n";
		std::cout << "Right: " << rightDistance << "\n";

		if (action == 0) {
			forward(&alphabot, 0.3);
		} else if (action == 1) {
			turnLeft(&alphabot, 0.3);
		} else if (action == 2) {
			turnRight(&alphabot, 0.3);
		}
	}

	void setAction(unsigned action) {
		this->action = action;
	}

private:
	unsigned action = 0;
	float leftDistance = 0;
	float rightDistance = 0;
	float leftDistanceLast = 0;
	float rightDistanceLast = 0;
	const float max = 0.1425;

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
		updateDistance(speed, speed);
	}

	void turnLeft(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(0.0);
		updateDistance(speed, 0.0);
	}

	void turnRight(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(speed);
		updateDistance(0.0, speed);
	}

	void updateDistance(float L, float R) {
		// 0.1425 is measure max speed
		float speedL = max * L; 
		float speedR = max * R; 

		// sampling rate 100ms
		leftDistance += speedL * 0.1;
		rightDistance += speedR * 0.1;

		leftDistanceLast = leftDistance;
		rightDistanceLast = rightDistance;

		float delta_distance = (leftDistance + rightDistance) / 2.0; 
  		float delta_theta = (rightDistance - leftDistance) / 0.142 % 3.14; // in radians

  		std::cout << "Delta distance: " << delta_distance << "\n";
  		std::cout << "Delta theta: " << delta_theta << "\n";
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
				action = 1;
			} else if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {
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
		// actions should be a set of waypoints [x, y, theta]
		unsigned action;
		action = data.getAction();
		control.setAction(action);
	}

	alphabot.stop();
	lidar.stop();
	return 0;
}
