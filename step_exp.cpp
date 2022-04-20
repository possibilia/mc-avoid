#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <curl/curl.h>
#include <list>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (action == 0) {
			forward(&alphabot);
		} else if (action == 1) {
			turnLeft(&alphabot);
		} else if (action == 2) {
			turnRight(&alphabot);
		} else {
			stop(&alphabot);
		}

		if (action == 1) {
			action = -1; 
		} else if (action == -1) {
			action = 1;
		}
	}

private:
	int action = 1;
	float speed = 0.2;

	void forward(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turnLeft(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turnRight(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(speed);
	}

	void stop(AlphaBot* alphabot) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
	}

};


int main(int, char **) { 
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	do {
	} while(!getchar());

	alphabot.stop();
	return 0;
}