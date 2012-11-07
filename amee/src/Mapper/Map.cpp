#include "Map.h"
#include <iostream>
#include "WallSegment.h"
#include "HorizontalWallSegment.h"
#include "VerticalWallSegment.h"

using namespace amee;

Map::Map() {
}

Map::~Map() {
	for (unsigned int i = 0; i < mWalls.size(); ++i) {
		delete mWalls[i];
	}
}

void Map::addMeasurement(Point pos) {
	bool belongsToWall = false;
	for (unsigned int i = 0; i < mWalls.size(); ++i) {
		belongsToWall = belongsToWall || mWalls[i]->addMeasurement(pos);
	}
	if (!belongsToWall) {
		WallSegment* horizontalWall = new HorizontalWallSegment(pos);
		WallSegment* verticalWall = new VerticalWallSegment(pos);
		mWalls.push_back(horizontalWall);
		mWalls.push_back(verticalWall);
	}
	//TODO tidy up
}

void Map::print() {
	std::cout << "Map has " << mWalls.size() << " walls" << "\n";
}