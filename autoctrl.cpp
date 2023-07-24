#include "agent.h"
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
class A1LidarScanEvent : public A1Lidar::DataInterface {
public:
    Agent& agent;

    A1LidarScanEvent(Agent& _agent) : agent(_agent) {};

public:
    void newScanAvail(float rpm, A1LidarData (&data)[A1Lidar::nDistance]) {
        int nData = 0;

        vector<Observation> obs;
        Observation ob; // invalid by default

        for(A1LidarData &data: data) {
            if (data.valid && data.r > safeDistance 
                && data.r < maxDetectRange) {
                ob.setObservation(data.x, data.y);
                nData++;
            } 
            obs.push_back(ob);
        }
        agent.eventNewRelativeCoordinates(rpm/60.0f, obs);
    }
};

// closes the loop
// callback to robot actuators from current task
// initiated by actor event handler
class MotorActionEvent : public ActionInterface {
public:
    MotorActionEvent(AlphaBot& _alphabot) : alphabot(_alphabot) {}

    AlphaBot& alphabot;
    virtual void executeMotorAction(float l, float r) {
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

    Agent agent;

    A1LidarScanEvent data(agent);
    A1Lidar lidar;

    lidar.registerInterface(&data);
    lidar.start();

    AlphaBot alphabot;
    MotorActionEvent takeAction(alphabot);

    shared_ptr<AbstractTask> targetTask = make_shared<StraightTask>();
    targetTask->registerInterface(&takeAction);
    targetTask->setInitialSpeed(0.2f);
    agent.setTargetTask(targetTask);

    alphabot.start();

    logger.startLogging("../test/manualctrl.txt", true);
    
    while(running) {
        // do nothing
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
