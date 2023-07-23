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
const float maxDetectRange = 2.0; 

const float rThresh = 0.05;
const float phiThresh = 0.1;
const float xThresh = 0.01;

const float motorSpeed = 0.3;
const float tolerance = 0.0250010341405868;

bool running = true;

const char* experiment = "2";

//Logger logger;

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
// 8192 data points 
// (invalid: corrupted/outside detection zone)
// scan event -> actor event handler
class DataInterface : public A1Lidar::DataInterface {
public:
    Actor& actor;

    DataInterface(Actor& _actor) : actor(_actor) {};

public:
    void newScanAvail(float rpm, A1LidarData (&data)[A1Lidar::nDistance]) {
        int nData = 0;

        vector<ObjectIdentifier> objects;
        ObjectIdentifier object; // invalid by default

        for(A1LidarData &data: data) {
            if (data.valid && data.r > safeDistance 
                && data.r < maxDetectRange) {
                object.setObject(data.x, data.y);
                nData++;
            } 
            objects.push_back(object);
        }
        actor.eventNewRelativeCoordinates(rpm/60.0f, objects);
        //logger.printf("RPM = %f, Samplerate = %f\n", rpm, rpm/60.0f);
    }
};

// closes the loop
// callback to robot actuators from task event
// initiated by a scan event from the lidar
class RoboTakeAction : public TakeAction {
public:
    RoboTakeAction(AlphaBot& _alphabot) : alphabot(_alphabot) {}

    AlphaBot& alphabot;
    virtual void motorAction(float l, float r) {
        //if (AIon) {
            alphabot.setLeftWheelSpeed(l);
            alphabot.setRightWheelSpeed(r);
        //}
    }
};

int main(int, char **) { 
    signal(SIGINT, sig_handler);

    Actor actor;

    DataInterface data(actor);
    A1Lidar lidar;

    lidar.registerInterface(&data);
    lidar.start();

    AlphaBot alphabot;
    RoboTakeAction takeAction(alphabot);

    shared_ptr<AbstractTask> targetTask = make_shared<StraightTask>();
    targetTask->registerTakeAction(&takeAction);
    actor.setTargetTask(targetTask);

    alphabot.start();

    logger.startLogging("../test/manualctrl.txt", true);
    
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
