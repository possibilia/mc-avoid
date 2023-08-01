#include "agent.h"
#include "alphabot.h"
#include "a1lidarrpi.h"
#include <vector>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <ncurses.h>

using namespace std;

const float safeDistance = 0.15; 
const float maxDetectRange = 1.0; 

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
        for(A1LidarData &data: data) {
            Observation ob; // invalid by default
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

    // set up planner and initilize model
    shared_ptr<StateMachineLTL> planner = make_shared<StateMachineLTL>(15);
    planner->setTransition(0, 1, make_shared<Rotate90Left>());
    planner->setTransition(0, 2, make_shared<Rotate90Right>());
    planner->setTransition(1, 3, make_shared<StraightTask>());
    planner->setTransition(2, 4, make_shared<StraightTask>());
    planner->setTransition(3, 5, make_shared<Rotate90Right>());
    planner->setTransition(5, 7, make_shared<StraightTask>());
    planner->setTransition(3, 9, make_shared<Rotate90Left>());
    planner->setTransition(9, 11, make_shared<StraightTask>());
    planner->setTransition(4, 6, make_shared<Rotate90Left>());
    planner->setTransition(6, 8, make_shared<StraightTask>());
    planner->setTransition(4, 10, make_shared<Rotate90Right>());
    planner->setTransition(10, 12, make_shared<StraightTask>());
    planner->setTransition(2, 13, make_shared<Rotate90Right>());
    planner->setTransition(1, 13, make_shared<Rotate90Left>());
    planner->setTransition(13, 14, make_shared<StraightTask>());

    targetTask->registerInterface(&takeAction);
    targetTask->setInitialSpeed(0.8f);

    agent.setTargetTask(targetTask);
    agent.setPlanner(planner);

    alphabot.start();

    logger.startLogging("../test/manualctrl.txt", true);
    
    while(running) {
        // blocking
        int ch = getchar();
        switch (ch) {
            case 27:
                running = false;
                break;
            default:
                 break;
        }
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
