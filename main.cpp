#include "alphabot.h"
#include <thread>

void forward(AlphaBot* alphabot, float speed);

void turn(AlphaBot* alphabot, float speed);

int main(int, char **) {
	AlphaBot alphabot;
	alphabot.start();

	// start test
	forward(&alphabot, 0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	turn(&alphabot, 0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	forward(&alphabot, 0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	// stop thread
	alphabot.stop();

	// exit 
	return 0;
}

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
