#include "alphabot.h"
#include "control.h"
#include <thread>

void north(AlphaBot alphabot, float speed);

void south(AlphaBot alphabot, float speed);

int main(int, char **) {
	AlphaBot alphabot;
	alphabot.start();

	Control control(&alphabot);

	// start test
	north(alphabot, 0.2)
	std::this_thread::sleep_for(std::chrono::seconds(5));

	south(alphabot, 0.2)
	std::this_thread::sleep_for(std::chrono::seconds(5));

	alphabot.stop();

	// exit 
	return 0;
}

void north(AlphaBot alphabot, float speed) {
	alphabot.setLeftWheelSpeed(speed);
	alphabot.setRightWheelSpeed(speed);
}

void south(AlphaBot alphabot, float speed) {
	alphabot.setLeftWheelSpeed(-speed);
	alphabot.setRightWheelSpeed(-speed);
}
