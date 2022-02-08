#ifndef CONTROL_H
#define CONTROL_H

class Control {
public:

	Control(AlphaBot* alphabot) {
		this->alphabot = alphabot;
	}

	~Control() {
		delete this;
	}

	void forward(float speed);

	void stop();

	void turn();

private:

	AlphaBot* alphabot;

};

#endif