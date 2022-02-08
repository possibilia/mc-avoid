#include "alphabot.h"
#include "a1lidarrpi.h"
#include "control.h"
// #include <thread>

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float,A1LidarData (&data)[A1Lidar::nDistance]) {
		for(A1LidarData &data: data) {
			if (data.valid & data.r < 0.2 & data.r > 0.0) {
				control.turn(0.2);
			}
			else {
				control.forward(0.2);
			}

		}
	}

	AlphaBot alphabot;
	Control control(&alphabot);
};

int main(int, char **) {
	// perception
	A1Lidar lidar;
	DataInterface dataInterface;
	lidar.registerInterface(&dataInterface);
	lidar.start();

	do {
	} while (!getchar());

	lidar.stop();
	fprintf(stderr,"\n");
}