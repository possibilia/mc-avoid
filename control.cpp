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
		if (weights[0] < 0.3 || weights[1] < 0.3) {
			evade(&alphabot);
			resetProgress();
		}

		float targetTheta = action_q.front()[0];
		float targetDistance = action_q.front()[1];

		if ((deltaTheta < targetTheta) & (targetTheta > 0)) {
			turnLeft(&alphabot, deltaTheta / targetTheta);
		} else if ((deltaTheta < targetTheta) & (targetTheta < 0)) {
			turnRight(&alphabot, deltaTheta / targetTheta);
		} else if (deltaDistance < targetDistance) {
			drive(&alphabot);
		} else {
			action_q.pop_front();
			resetProgress();
		}
	}

	void setWeights(std::vector<float> weights) {
		this->weights = weights;
	}

	void setActions(std::vector<std::vector<float>> actions) {
		action_q.clear();
		for (std::vector<float> row : actions) {
			action_q.push_back(row);
		}
	}

private:
	const float speed = 0.5;
	const float tol = 0.1;
	const float L = 0.142 * 0.865;
	const float actualSpeedMax = 0.2;
	const float samplingRate = 0.1; 

	std::vector<float> weights = {1.0, 1.0}; 
	std::list<std::vector<float>> action_q = {};

	float leftDistance = 0;
	float rightDistance = 0;

	float deltaDistance = 0;
	float deltaTheta = 0;

	void drive(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turnLeft(AlphaBot* alphabot, float thetaPercent) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed * thetaPercent);
		updateProgress(speed, speed * thetaPercent);
	}

	void turnRight(AlphaBot* alphabot, float thetaPercent) {
		alphabot->setLeftWheelSpeed(speed * thetaPercent);
		alphabot->setRightWheelSpeed(speed);
		updateProgress(speed * thetaPercent, speed);
	}

	void evade(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(speed * weights[1]);
		alphabot->setRightWheelSpeed(speed * weights[0]);
	}

	void updateProgress(float scaledSpeedLeft, float scaledSpeedRight) {
		leftDistance += actualSpeedMax * scaledSpeedLeft * samplingRate;
		rightDistance += actualSpeedMax * scaledSpeedRight * samplingRate;

		deltaDistance = (leftDistance + rightDistance) / 2.0; 
  		deltaTheta = (rightDistance - leftDistance) / L; 
	}

	void resetProgress() {
		leftDistance = 0;
		rightDistance = 0;

		deltaDistance = 0;
		deltaTheta = 0;
	}
};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		weights = {1.0, 1.0};

		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < rmax) & (data.r > 0.0) & 
				(data.phi > 0.5) & (data.phi < 1.5708)) {
				if (data.r < weights[0]) {
					weights[0] = data.r;
				}
			} else if ((data.valid) & (data.r < rmax) & (data.r > 0.0) & 
				(data.phi < -0.5) & (data.phi > -1.5708)) {
				if (data.r < weights[1]) {
					weights[1] = data.r;
				}
			}
		}
	}

	std::vector<float> getWeights() {	
		return weights;
	}

	std::vector<std::vector<float>> getActions() {
		return actions;
	}

private:
	const float rmax = 1.0;
	const float rmin = 0.0;

	std::vector<float> weights = {1.0, 1.0};
	std::vector<std::vector<float>> actions = {
		{1.5708, 0.2},
		{-1.5708, 0.2},
		{1.5708, 0.2},
		{-1.5708, 0.2}
	};
};

int main(int, char **) { 
	DataInterface data;
	ControlCallback control;

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	std::vector<std::vector<float>> actions;
	actions = data.getActions();
	control.setActions(actions);

	while(true) {
		std::vector<float> weights;
		std::vector<std::vector<float>> actions;

		weights = data.getWeights();
		actions = data.getActions();

		control.setActions(actions);
		control.setWeights(weights);
	}

	alphabot.stop();
	return 0;
}
