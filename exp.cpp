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

	void pushActions(float* actions, bool front) {
		unsigned rows = sizeof(&actions) / sizeof(&actions[0]);

		for (unsigned i = 0; i < rows; i++) {
			if (front) {
				action_q.push_front(&actions[i]);
			} else {
				action_q.push_back(&actions[i]);
			}
		}
	}

private:
	// wheel separation m
	const float L = 0.142;

	// max speed m/s
	const float actualSpeedMax = 0.228;

	// sampling rate s
	const float samplingRate = 0.1;

	// maintained queue of actions
	std::list<float[]> action_q = {};

	float leftDistance = 0;
	float rightDistance = 0;

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
		if (theta < 0) { // turn right
			alphabot->setLeftWheelSpeed(speed);
			alphabot->setRightWheelSpeed(-speed);
			updateDistance(speed, -speed);

		} else { // turn left
			alphabot->setLeftWheelSpeed(-speed);
			alphabot->setRightWheelSpeed(speed);
			updateDistance(-speed, speed);
		}
	}

	void updateDistance(float scaledSpeedLeft, float scaledSpeedRight) {
		leftDistance += actualSpeedMax * scaledSpeedLeft * samplingRate;
		rightDistance += actualSpeedMax * scaledSpeedRight * samplingRate;

		delta_distance = (leftDistance + rightDistance) / 2.0; 
  		delta_theta = (rightDistance - leftDistance) / L; 
	}

};


int main(int, char **) { 
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	// distances and angles
	float actions[][] = {
		{0.2, 1.5708},
		{0.2, 1.5708},
		{0.2, 1.5708},
		{0.2, 1.5708},
	};

	control.pushActions(actions, false);

	while(true) {
		// do nothing
	}

	alphabot.stop();
	return 0;
}
