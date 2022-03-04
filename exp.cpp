#include "alphabot.h"
#include <thread>
#include <string>
#include <iostream>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		unsigned t = 0;

		if (t == 0) {
			turn(&alphabot, speed);
			t = 1;
		} else {
			forward(&alphabot, speed);
			t = 0;
		}
	}

	void setSpeed(float speed) {
		this->speed = speed;
	}

private:
	float speed = 0.0;

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turn(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(0.0);
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
		command = std::cin >> "q";

		if (command == "q") {
			alphabot.stop();
			lidar.stop();
			break;
		}
	}

	return 0;
}