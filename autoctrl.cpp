#include "lib.h"
#include "alphabot.h"
#include "a1lidarrpi.h"
#include <vector>
#include <signal.h>
#include <iostream>
#include <stdio.h>

using namespace std;

const float safeDistance = 0.15; 
const float maxDetectRange = 2.0; 

const float rThresh = 0.05;
const float phiThresh = 0.1;
const float xThresh = 0.01;

const float motorSpeed = 0.3;
const float tolerance = 0.0250010341405868;

bool running = true;

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
    }
};

// closes the loop
// callback to robot actuators from current task
// initiated by actor event handler
class ActuatorInterface : public TakeAction {
public:
    ActuatorInterface(AlphaBot& _alphabot) : alphabot(_alphabot) {}

    AlphaBot& alphabot;
    virtual void motorAction(float l, float r) {
        alphabot.setLeftWheelSpeed(l);
        alphabot.setRightWheelSpeed(r);
    }
};

// terminate sig handler
void sig_handler(int signo) {
    if (signo == SIGINT) {
            running = false;
    }
}

int main(int, char **) { 
    signal(SIGINT, sig_handler);

    Actor actor;

    DataInterface data(actor);
    A1Lidar lidar;

    lidar.registerInterface(&data);
    lidar.start();

    AlphaBot alphabot;
    ActuatorInterface takeAction(alphabot);

    shared_ptr<AbstractTask> targetTask = make_shared<StraightTask>();
    targetTask->registerTakeAction(&takeAction);
    actor.setTargetTask(targetTask);

    alphabot.start();

    logger.startLogging("../test/manualctrl.txt", true);
    
    while(running) {
        // do nothing
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
