#include "alphabot.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		stop(&alphabot, speed);

		for (int i = 0; i < 5; i++) {
			turn(&alphabot, speed);
		}
	}
	
	void setSpeed(float speed) {
		this->speed = speed;
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

	control.setSpeed(0.2);

	while(true) {
		std::string command;
		std::cin >> command;

		if (command == "q") {
			alphabot.stop();
			break;
		}
	}

	return 0;
}s
