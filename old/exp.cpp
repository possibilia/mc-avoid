#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <cmath>
#include <curl/curl.h>
#include <list>
#include <vector>

struct {
	std::vector<float> DRIVE_AHEAD = {0.2, 0.0};
	std::vector<float> TURN_LEFT = {0.2, 1.570796};
	std::vector<float> TURN_RIGHT = {0.2, -1.570796};
	std::vector<float> SWERVE_LEFT = {0.2, 1.570796 / 2};
	std::vector<float> SWERVE_RIGHT = {0.2, -1.570796 / 2};
	std::vector<float> TURN_AROUND = {0.0, 1.570796 * 2};
} localActions; 

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		bool distanceReached = delta_distance < action_q.front()[0];
		bool angleReached = std::abs(delta_theta) < std::abs(action_q.front()[1]);

		if ((action_q.front()[0] == 0.0) & angleReached) {
			turn_around(&alphabot, 0.3);
		} else if (angleReached) {
			turn(&alphabot, 0.3);
		} else if (distanceReached) {
			forward(&alphabot, 0.3);
		} else {
			action_q.pop_front();
			
			leftDistance = 0;
			rightDistance = 0;

			delta_distance = 0;
			delta_theta = 0;
		}
	}

	void pushActions(std::vector<std::vector<float>> actions) {
		for (std::vector<float> row : actions) {
			action_q.push_back(row);
		}
	}

private:
	const float tol = 0.1;
	const float L = 0.142 * 0.865;
	const float actualSpeedMax = 0.2;
	const float samplingRate = 0.1;

	std::list<std::vector<float>> action_q = {};

	float leftDistance = 0;
	float rightDistance = 0;

	float delta_distance = 0;
	float delta_theta = 0; 

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
		updateDistance(speed, speed);
	}

	void turn(AlphaBot* alphabot, float speed) {
		if (action_q.front()[1] < 0) { // turn right
			alphabot->setLeftWheelSpeed(speed+tol);
			alphabot->setRightWheelSpeed(-tol);
			updateDistance(speed+tol, -tol);
		} else { // turn left
			alphabot->setLeftWheelSpeed(-tol);
			alphabot->setRightWheelSpeed(speed+tol);
			updateDistance(-tol, speed+tol);
		}
	}

	void turn_around(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(-speed);
		alphabot->setRightWheelSpeed(speed);
		updateDistance(-speed, speed);
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
	std::vector<std::vector<float>> sequence = {
		localActions.DRIVE_AHEAD,
		localActions.TURN_AROUND,
		localActions.SWERVE_LEFT,
		localActions.DRIVE_AHEAD,
		localActions.SWERVE_RIGHT,
		localActions.DRIVE_AHEAD,
		localActions.TURN_RIGHT,
		localActions.TURN_AROUND,
		localActions.DRIVE_AHEAD
	};

	control.pushActions(sequence);

	while(true) {
		// do nothing
	}

	alphabot.stop();
	return 0;
}
