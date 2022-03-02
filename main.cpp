#include "alphabot.h"
#include "control.h"
#include <thread>

int main(int, char **) {
	AlphaBot alphabot;
	alphabot.start();

	Control control(&alphabot);

	// start test
	alphabot.setLeftWheelSpeed(0.2);
	alphabot.setRightWheelSpeed(0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	alphabot.setLeftWheelSpeed(0.0);
	alphabot.setRightWheelSpeed(0.0);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	alphabot.setLeftWheelSpeed(-0.2);
	alphabot.setRightWheelSpeed(0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	alphabot.setLeftWheelSpeed(0.0);
	alphabot.setRightWheelSpeed(0.0);
	alphabot.stop();

	// exit 
	return 0;
}