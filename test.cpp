#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <curl/curl.h>
#include <boost/format.hpp>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
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

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turnLeft(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turnRight(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(speed);
	}

};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		// setup curl for data
		auto curl = curl_easy_init();

		// reset action
		if ((action == 1) || (action == 2)) {
			action = 0;
		}

		// determine action
		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi > 0.0) & (data.phi < 1.0)) {
				action = 1;
			} else if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {
				action = 2;
			}
			
			// get timestamp using unix epoch (secs)
			auto now = std::chrono::system_clock::now();
			unsigned t = std::chrono::duration_cast<std::chrono::seconds>(
						 now.time_since_epoch()).count();

			// send data to server
			boost::format query = boost::fromat(URL) % t, data.x, 
								  data.y, data.r, data.phi, data.signal;
			curl_easy_setopt(curl, query);
		}
		
		// tear down curl for data
		curl_easy_perform(curl);
		curl_easy_cleanup(curl);
        curl = NULL;
	}

	unsigned getAction() {	
		return action;
	}

private:
	unsigned action = 0;
	const std::string URL = "https://localhost/query?t=%u&x=%f&y=%f&r=%f&phi=%f&signal=%f";

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
		unsigned action;
		action = data.getAction();
		control.setAction(action);
	}

	alphabot.stop();
	lidar.stop();
	return 0;
}
