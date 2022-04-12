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
		alphabot.setLeftWheelSpeed(speed * weights[1]);
		alphabot.setRightWheelSpeed(speed * weights[0]);
	}

	void setWeights(std::vector<float> weights) {
		this->weights = weights;
	}

private:
	const float speed = 0.5;
	std::vector<float> weights = {1.0, 1.0}; 
};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		weights = {1.0, 1.0};

		for(A1LidarData &data: data) {
			updateGrid(data.x, data.y);
			if ((data.valid) & (data.r < rmax) & (data.r > 0.0) & 
				(data.phi > 0.0) & (data.phi < 1.0)) {

				float r = rescale(data.r)
				if (r < weights[0]) {
					weights[0] = r;
				}

			} else if ((data.valid) & (data.r < rmax) & (data.r > 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {

				float r = rescale(data.r);
				if (r < weights[1]) {
					weights[1] = r;
				}
			}
		}
	}

	std::vector<float> getWeights() {	
		return weights;
	}
	
private:
	const float rmax = 2.0;
	const float rmin = 0.2;

	std::vector<float> weights;

	float rescale(float r) {
		return ((r - rmax) / (rmax - rmin)) * 1.0;
	}
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

	while(true) {
		std::vector<float> weights;
		weights = data.getWeights();
		control.setWeights(weights);
	}

	alphabot.stop();
	return 0;
}
