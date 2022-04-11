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
	// has to be the pose so need kinematics
	// also has to be options to swerve robot
	std::vector<float> DRIVE_AHEAD = {0.2, 0.0};
	std::vector<float> DRIVE_LEFT45 = {0.2, 0.785398};
	std::vector<float> DRIVE_LEFT90 = {0.2, 0.785398 * 2};
	std::vector<float> DRIVE_LEFT135 = {0.2, 0.785398 * 3};
	std::vector<float> DRIVE_LEFT180 = {0.2, 0.785398 * 4};
	std::vector<float> DRIVE_RIGHT135 = {0.2, -0.785398 * 3};
	std::vector<float> DRIVE_RIGHT90 = {0.2, -0.785398 * 2};
	std::vector<float> DRIVE_RIGHT45 = {0.2, -0.785398};
} localActions; 

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (std::abs(delta_theta) < std::abs(action_q.front()[1])) {
			turn(&alphabot, 0.3);
		} else if (delta_distance < action_q.front()[0]) {
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
	// wheel separation m
	const float L = 0.142 * 0.865;

	// max speed m/s
	const float actualSpeedMax = 0.2;

	// sampling rate s
	const float samplingRate = 0.1;

	// maintained queue of actions
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
	std::vector<std::vector<float>> sequence = {
		localActions.DRIVE_AHEAD,
		localActions.DRIVE_LEFT90,
		localActions.DRIVE_LEFT90,
		localActions.DRIVE_LEFT90,
		localActions.DRIVE_LEFT90,
	};

	control.pushActions(sequence);

	while(true) {
		// do nothing
	}

	alphabot.stop();
	return 0;
}
