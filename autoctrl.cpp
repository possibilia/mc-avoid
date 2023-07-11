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
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		// organize/filter data
		vector<A1LidarData> points;
		for(A1LidarData &data: data) {
			if (data.valid && data.r > safeDistance 
				&& data.r < maxDetectRange) {
				points.push_back(data);
			}
		}

		// changepoint indices
		vector<int> cpIdx = {0};
		for(unsigned i = 1; i < points.size(); ++i) {
			float rDiff = points[i].r - points[i-1].r;
			float phiDiff = points[i].phi - points[i-1].phi;

			if (abs(rDiff) > rThresh || abs(phiDiff) > phiThresh) {
				cpIdx.push_back(i-1);
				cpIdx.push_back(i);
			} 
		}
		cpIdx.push_back(points.size()-1);

		// changepoints in matrix for now
		// ultimately need an object class
		// and we ned to reliably identify 
		// the actual line segments here
		// how do we control for single points?
		vector<vector<float>> lineSegments;
		for(unsigned i = 0; i < cpIdx.size(); ++i) {
			vector<float> v = {points[cpIdx[i]].x, points[cpIdx[i]].y};
			lineSegments.push_back(v);
		}

		// clocks in around 0.650 ms
		// save all points
		saveMap(points);
		nScan++;
	}

private:
	void saveMap(const std::vector<A1LidarData>& objects) {
		char tmp[256];
		// need to be able to specify the run folder
		sprintf(tmp,"../1/map%03d.dat", nScan);
		fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		for(unsigned i = 0; i < objects.size();i++) {
			fprintf(f,"%4.4f %4.4f\n",
				objects[i].x,
				objects[i].y);
		}
		fclose(f);
	}

	unsigned nScan = 0;
};

int main(int, char **) { 
	signal(SIGINT, sig_handler);
	DataInterface data;
	A1Lidar lidar;

	lidar.registerInterface(&data);
	lidar.start();

	// logger.startLogging("autoctrl.txt", true);
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
