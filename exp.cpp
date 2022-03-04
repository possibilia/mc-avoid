#include "alphabot.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		stop(&alphabot, speed);
		turn(&alphabot, speed);

		if (speed > 1.0) {
			speed = 0.0;
		} else {
			speed += 0.1;
		}
	}

private:
	float speed = 0.0;

	void stop(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turn(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(-speed);
	}
};

int main(int, char **) { 
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	while(true) {
		std::string command;
		std::cin >> command;

		if (command == "q") {
			alphabot.stop();
			break;
		}
	}

	return 0;
}
