#include "lib.h"

int nEvents = 0;

void Actor::eventNewRelativeCoordinates(
	const vector<ObjectIdentifier>& objects) {

	nEvents++;
	saveMap(objects);

}