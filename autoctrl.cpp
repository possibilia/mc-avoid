#include "a1lidarrpi.h"
#include <signal.h>

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		for(A1LidarData &data: data) {
			// TODO
		}
	}
};

void signal_callback_handler(int signum) {
   exit(signum);
}

int main(int, char **) { 
	DataInterface data;
	A1Lidar lidar;

	lidar.registerInterface(&data);
	lidar.start();

	signal(SIGINT, signal_callback_handler);

	while(true) {
		// do nothing
	}

	return 0;
}