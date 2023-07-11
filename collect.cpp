#include "alphabot.h"
#include "a1lidarrpi.h"
#include <signal.h>

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		if (scan) {
			for(A1LidarData &data: data) {
				if (data.valid) {
					std::cout << data.x << "\t" << data.y << "\t" << data.r << "\t" << data.phi << "\n";
				}
			
			}
			scan = false;
		}
		
	}

private:
	bool scan = true;
};

int main(int, char **) { 
	std::cout << "x\ty\tr\tphi\n";

	DataInterface data;

	A1Lidar lidar;
    lidar.registerInterface(&data);
    lidar.start();

	while(true) {
		// do nothing
	}

	return 0;
}
