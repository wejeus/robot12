#include "Map.h"

using namespace amee;

Map::~Map() {
	for (unsigned int i = 0; i < mWalls.size(); ++i) {
		delete mWalls[i];
	}
}

void Map::addMeasurement(Point pos) {
	bool belongsToWall = false;
	for (unsigned int i = 0; i < mWalls.size(); ++i) {
		belongsToWall = belongsToWall || mWalls[i].addMeasurement(pos);
	}
	if (!belongsToWall) {
		WallSegment* hori;
		mWalls.push_back()
	}
}