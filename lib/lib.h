#ifndef control_h
#define control_h

#include "logger.h"
#include <vector>
#include <cassert>

using namespace std;

extern int nEvents;

class Point {
public:
	float x = 0.0;
	float y = 0.0;
	
	Point() {}

	Point(float x, float y) {
		set(x, y);
	}

	void set(float x, float y) {
		this->x = x;
		this->y = y;
	}
};

class ObjectIdentifier {
public:

	ObjectIdentifier() {}

	ObjectIdentifier(float x, float y) {
		setLocation(x, y);
	}

	void setLocation(float x, float y) {
		Point l(x, y);
		location = l;
	}

	Point getLocation() const {
		return location;
	}

private:
	Point location;
};

class Actor {
public:
	void eventNewRelativeCoordinates(
		const vector<ObjectIdentifier>& objects);

private:
	void saveMap(const vector<ObjectIdentifier>& objects) {
		char tmp[256];
		sprintf(tmp,"../test/map%03d.dat",nEvents);
		fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		for(unsigned i = 0; i < objects.size();i++) {
			//if (objects[i].isValid()) {
				fprintf(f,"%4.4f %4.4f\n",
				objects[i].getLocation().x,
				objects[i].getLocation().y);
			//}
		}
		fclose(f);
	}
};


#endif