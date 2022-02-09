#include "control.h"

void Control::forward(float speed) {
	alphabot->setLeftWheelSpeed(speed);
	alphabot->setRightWheelSpeed(speed);
}

void Control::stop() {
	alphabot->setLeftWheelSpeed(0);
	alphabot->setRightWheelSpeed(0);
}

void Control::turn(float speed) {
	alphabot->stop();
	alphabot->setLeftWheelSpeed(speed);
	alphabot->setRightWheelSpeed(-speed);
}