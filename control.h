#ifndef CONTROL_H
#define CONTROL_H

class Control {
public:

	Control(AlphaBot* alpha) {
		this->alphabot = alpha;
	}

	~Control() {
		delete this;
	}

	void forward(float speed);

	void stop();

	void turn(float speed);

private:

	AlphaBot* alphabot;

};

#endif