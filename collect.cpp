#include "alphabot.h"
#include "a1lidarrpi.h"

using namespace std;

const float minDetectRange = 0.15; 
const float maxDetectRange = 1.0; 

bool running = true;

// every 200 ms
// 8192 data points 
// (invalid: corrupted/outside detection zone)
// scan event -> actor event handler
class A1LidarScanEvent : public A1Lidar::DataInterface {
public:
    AlphaBot& agent;

    A1LidarScanEvent(AlphaBot& _agent) : agent(_agent) {};

public:
    void newScanAvail(float rpm, A1LidarData (&data)[A1Lidar::nDistance]) {
        int nData = 0;

        for(A1LidarData &data: data) {
            if (data.valid) {
                cout << data.x << "\t" << data.y << "\t" << data.r << "\t" << data.phi << "\n";
                nData++;
            }
        }
    }
};


int main(int, char **) { 

    cout << "x\ty\tr\tphi\n";

    AlphaBot alphabot;

    A1LidarScanEvent data(alphabot);
    A1Lidar lidar;

    lidar.registerInterface(&data);
    lidar.start();

    alphabot.start();
    
    while(running) {
        // blocking
        int ch = getchar();
        switch (ch) {
            case 27:
                running = false;
                break;
            case 'a':
                alphabot.setLeftWheelSpeed(-0.8);
                alphabot.setRightWheelSpeed(0.8);
                break;
            case 'd':
                alphabot.setLeftWheelSpeed(0.8);
                alphabot.setRightWheelSpeed(-0.8);
                break;
            case 'w':
                alphabot.setLeftWheelSpeed(0.8);
                alphabot.setRightWheelSpeed(0.8);
                break;
            default:
                break;
        }
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
