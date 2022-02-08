#ifndef NAVIGATION_H
#define NAVIGATION_H

class Navigation {
public:

	Navigation(float thresh) {
		this->thresh = thresh; 
	}

	~Navigation() {
		delete this;
	}

	int getActions(A1LidarData &data);

private:

	float thresh;

};

#endif
