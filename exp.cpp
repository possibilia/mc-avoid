#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <cmath>
#include <curl/curl.h>
#include <list>
#include <vector>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (std::abs(delta_theta) < std::abs(action_q.front()[1])) {
			turn(&alphabot, 0.3);
		} else if (delta_distance < action_q.front()[0]) {
			forward(&alphabot, 0.3);
			action_q.pop_front();
		}
		else if (action_q.empty()){
			stop(&alphabot);
		}
	}

	void pushActions(std::vector<std::vector<float>> actions, bool front) {
		for (std::vector<float> row : actions) {
			if (front) {
				action_q.push_front(row);
			} else {
				action_q.push_back(row);
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
	std::list<std::vector<float>> action_q = {};

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
		if (action_q.front()[1] < 0) { // turn right
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
	std::vector<std::vector<float>> actions = {
		{0.2, 1.5708},
		{0.2, 1.5708},
		{0.2, 1.5708},
		{0.2, 1.5708},
	};

	control.pushActions(actions, false);

	while(true) {
		// do nothing
		std::cout << "Working" << "\n";
	}

	alphabot.stop();
	return 0;
}
