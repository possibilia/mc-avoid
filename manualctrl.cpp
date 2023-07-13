#include "lib.h"
#include "alphabot.h"
#include "a1lidarrpi.h"
#include <signal.h>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <list>
//#include <string>

using namespace std;
//using namespace std::chrono;

const float safeDistance = 0.15; 
const float maxDetectRange = 0.5; 

const float rThresh = 0.05;
const float phiThresh = 0.1;
const float xThresh = 0.01;

const float motorSpeed = 0.3;

bool running = true;

const char* experiment = "2";

Logger logger;

// terminate sig handler
void sig_handler(int signo) {
        if (signo == SIGINT) {
                running = false;
        }
}

int getch(void) {
    int ch;

    struct termios oldt;
    struct termios newt;
  
    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt; 
    newt.c_lflag &= ~(ICANON | ECHO); 
  
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
    ch = getchar(); 
  
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
  
    return ch;
}


// every 200 ms
class DataInterface : public A1Lidar::DataInterface {
public:
    void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
        
        logger.printf("Scan %03d:\n", nScan);
        // organize/filter data
        vector<A1LidarData> points;
        for(A1LidarData &data: data) {
            if (data.valid && data.r > safeDistance 
                    && data.r < maxDetectRange) {
                    points.push_back(data);
            }
        }

        // changepoint indices
        vector<int> cpIdx = {};

        // test if a line intersects x-axis
        float xDiff = points[0].x - points[points.size()-1].x;
        bool intersectX = abs(xDiff) < xThresh;
        if (!intersectX) { cpIdx.push_back(0); }

        // detect changepoints
        for(unsigned i = 1; i < points.size(); ++i) {
            float rDiff = points[i].r - points[i-1].r;
            float phiDiff = points[i].phi - points[i-1].phi;

            if (abs(rDiff) > rThresh || abs(phiDiff) > phiThresh) {
                    cpIdx.push_back(i-1);
                    cpIdx.push_back(i);
            } 
        }
        //cpIdx.push_back(points.size()-1);
        if (!intersectX) { cpIdx.push_back(points.size()-1);}

        // collect changepoints
        vector<vector<float>> changepoints;
        for(unsigned i = 0; i < cpIdx.size(); ++i) {
            vector<float> v = {points[cpIdx[i]].x, points[cpIdx[i]].y};
            changepoints.push_back(v);
        }

        saveMap(points);
        saveLines(changepoints);
        nScan++;
    }

private:
    void saveMap(const vector<A1LidarData>& objects) {
        char tmp[256];
        // need to be able to specify the run folder
        sprintf(tmp,"../%s/map%03d.tsv", experiment, nScan);
        FILE* f = fopen(tmp,"wt");
        fprintf(f,"x\ty\n");
        for(unsigned i = 0; i < objects.size();i++) {
                fprintf(f,"%4.4f\t%4.4f\n",
                        objects[i].x,
                        objects[i].y);
        }
        fclose(f);
    }

    void saveLines(const vector<vector<float>>& changepoints) {
        char tmp[256];
        // need to be able to specify the run folder
        sprintf(tmp,"../%s/changepoints%03d.tsv", experiment, nScan);
        FILE* f = fopen(tmp,"wt");
        fprintf(f,"x\ty\n");
        for(unsigned i = 0; i < changepoints.size();i++) {
                fprintf(f,"%4.4f\t%4.4f\n",
                        changepoints[i][0],
                        changepoints[i][1]);
        }
        fclose(f);
    }

    unsigned nScan = 0;
};

int main(int, char **) { 
    signal(SIGINT, sig_handler);

    DataInterface data;
    A1Lidar lidar;
    AlphaBot alphabot;

    lidar.registerInterface(&data);
    lidar.start();
    alphabot.start();

    logger.startLogging("../2/manualctrl.txt", true);
    
    while(running) {
        int ch = getch();
        switch (ch) {
            case ' ':
                alphabot.setLeftWheelSpeed(0);
                alphabot.setRightWheelSpeed(0);
                logger.printf("STOP!\n");
                break;
            case 'a':
                alphabot.setLeftWheelSpeed(-motorSpeed);
                alphabot.setRightWheelSpeed(motorSpeed);
                logger.printf("Initiating turning left...\n");
                break;
            case 'd':
                alphabot.setLeftWheelSpeed(motorSpeed);
                alphabot.setRightWheelSpeed(-motorSpeed);
                logger.printf("Initiating turning right...\n");
                break;
            case 's':
                alphabot.setLeftWheelSpeed(-motorSpeed);
                alphabot.setRightWheelSpeed(-motorSpeed);
                logger.printf("Initiating move backwards...\n");
                break;
            case 'w':
                alphabot.setLeftWheelSpeed(motorSpeed);
                alphabot.setRightWheelSpeed(motorSpeed);
                logger.printf("Initiating moving forward...\n");
                break;
            default:
                break;
        }
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
