#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <cmath>
#include <curl/curl.h>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (std::abs(delta_theta) < std::abs(theta)) {
			turn(&alphabot, 0.3);
		} else if (delta_distance < distance) {
			forward(&alphabot, 0.3);
		} else {
			stop(&alphabot);
		}
	}

private:
	const float max = 0.1425;

	float distance = 0.2;
	float theta = 1.5708;

	float leftDistance = 0;
	float rightDistance = 0;

	float leftWheelSpeed = 0;
	float rightWheelSpeed = 0;

	float delta_distance;
	float delta_theta; 

	void stop(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
	}

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
		updateDistance(speed, speed);
	}

	void turn(AlphaBot* alphabot, float speed) {
		if (theta < 0) {
			// turn right if theta -ve
			leftWheelSpeed = speed;
			rightWheelSpeed = -speed;

		} else {
			// otherwise +ve so turn left
			leftWheelSpeed = -speed;
			rightWheelSpeed = speed;
		}

		alphabot->setLeftWheelSpeed(leftWheelSpeed);
		alphabot->setRightWheelSpeed(rightWheelSpeed);
		updateDistance(leftWheelSpeed, rightWheelSpeed);
	}

	void updateDistance(float L, float R) {
		// 0.1425 is measure max speed
		float speedL = max * L; 
		float speedR = max * R; 

		// sampling rate 100ms/0.1s
		leftDistance += speedL * 0.1;
		rightDistance += speedR * 0.1;

		delta_distance = (leftDistance + rightDistance) / 2.0; 
  		delta_theta = (rightDistance - leftDistance) / 0.14; // in radians
	}

};


int main(int, char **) { 
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	while(true) {
		// do nothing
	}

	alphabot.stop();
	return 0;
}
