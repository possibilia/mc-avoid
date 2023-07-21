#include "lib.h"
#include "alphabot.h"
#include "a1lidarrpi.h"
#include <signal.h>
#include <vector>
#include <typeinfo>
// #include<chrono>

using namespace std;
using namespace std::chrono;

const float safeDistance = 0.2; 
const float maxDetectRange = 1.0; 

const float rThresh = 0.05;
const float phiThresh = 0.1;

Logger logger;

bool running = true;

// terminate sig handler
void sig_handler(int signo) {
	if (signo == SIGINT) {
		running = false;
	}
}

// every 200 ms
class DataInterface : public A1Lidar::DataInterface {
public:
	Actor& actor;

	// wtf does this construction mean?
	DataInterface(Actor& _actor) : actor(_actor) {};

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		int nData = 0;

		vector<ObjectIdentifier> objects;
		for(A1LidarData &data: data) {
			// do we really want invalid data?
			// just create objects on the heap
			// which do not have any point info
			ObjectIdentifier object;
			if (data.valid && data.r > safeDistance 
				&& data.r < maxDetectRange) {
				object.setLocation(data.x, data.y);
				nData++;
			}
			objects.push_back(object);
		}
		actor.eventNewRelativeCoordinates(objects);
		logger.printf("nData = %d", nData);
	}
};

int main(int, char **) { 
	signal(SIGINT, sig_handler);
	Actor actor;

	DataInterface data(actor);
	A1Lidar lidar;

	lidar.registerInterface(&data);
	lidar.start();

	logger.startLogging("../test/autoctrl.txt", true);
	// TestLogger test;

	//AlphaBot alphabot;
	//alphabot.registerStepCallback(&control);
	//alphabot.start();

	//alphabot.setLeftWheelSpeed(-0.5);
	//alphabot.setRightWheelSpeed(0.5);

	while(running) {
		// do nothing
		// test.test("hello\n");
	}

	return 0;
}
