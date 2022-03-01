#include "alphabot.h"
#include "control.h"
#include <thread>

int main(int, char **) {
	AlphaBot alphabot;
	alphabot.start();

	Control control(&alphabot);

	// start test
	control.forward(0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	control.stop();
	std::this_thread::sleep_for(std::chrono::seconds(5));

	control.turn(0.2);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	control.stop();
	alphabot.stop();

	// exit 
	return 0;
}