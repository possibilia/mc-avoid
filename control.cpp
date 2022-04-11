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
	std::vector<float> DRIVE_AHEAD = {0.0, 0.0};
	std::vector<float> TURN_RIGHT = {0.0, 1.};
	std::vector<float> TURN_LEFT = {0.0, -1.};
} localActions; 

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (action_q.front()[1] == 0.0) {
			alphabot.setLeftWheelSpeed(speed);
			alphabot.setRightWheelSpeed(speed);
		} else {
			alphabot.setLeftWheelSpeed(speed);
			alphabot.setRightWheelSpeed(speed);
		}
		action_q.pop_front();
	}

	void pushActions(std::vector<std::vector<float>> actions) {
		for (std::vector<float> row : actions) {
			action_q.push_back(row);
		}
	}

private:
	const float speed = 0.3;
	std::list<std::vector<float>> action_q = {};

};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 1.0) & (data.r > 0.0) & 
				(data.phi > 0.0) & (data.phi < 1.0)) {
				if (data.r < weights[0]) {
					weights[0] = data.r;
				}
			} else if ((data.valid) & (data.r < 1.0) & (data.r > 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {
				if (data.r < weights[1]) {
					weights[1] = data.r;
				}
			}
		}
	}

	float * getWeights() {	
		return weights;
	}

private:
	float weights[] = {1.0, 1.0};
};

int main(int, char **) { 
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	// distances and angles
	std::vector<std::vector<float>> sequence = {
		localActions.DRIVE_AHEAD,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_LEFT, 
		localActions.TURN_LEFT,
		localActions.TURN_RIGHT, 
		localActions.TURN_RIGHT,
		localActions.DRIVE_AHEAD,
		localActions.DRIVE_AHEAD
	};

	control.pushActions(sequence);
	const float default_speed = 0.3;

	while(true) {
		float weights;
		weights = data.getWeights();
		localActions.TURN_LEFT[0] = default_speed * weights[0]
		localActions.TURN_RIGHT[0] = default_speed * weights[1];
	}

	alphabot.stop();
	return 0;
}
