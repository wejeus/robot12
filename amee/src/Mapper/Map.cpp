#include "Map.h"
#include <iostream>
#include "WallSegment.h"
#include "HorizontalWallSegment.h"
#include "VerticalWallSegment.h"

using namespace amee;

Map::Map() {
}

Map::~Map() {
	for (std::list<WallSegment*>::const_iterator iter = mWalls.begin(), end = mWalls.end(); iter != end; ++iter) {
		delete *iter;	
	}
}

void Map::reduceNumWalls(const Point& pos, float distance) {
	std::list<WallSegment*> closeHorizontals;
	std::list<WallSegment*> closeVerticals;

	for (std::list<WallSegment*>::iterator iter = mWalls.begin(), end = mWalls.end(); iter != end; ++iter) {
		WallSegment* wall = (*iter);
		float dist = wall->distanceTo(pos);
		if (dist < distance) {
			if (wall->getType() == WallSegment::HORIZONTAL) {
				closeVerticals.push_back(wall);
			} else {
				closeVerticals.push_back(wall);
			}
		} else if (wall->isSmall()) {
			iter = mWalls.erase(iter);
			--iter; // decrease iter so that we don't skip a wall
			delete wall;
		}	
	}

	// TODO merge walls
}

void Map::addMeasurement(Point pos) {
	bool belongsToWall = false;
	std::list<WallSegment*>::const_iterator iterator = mWalls.begin(), end = mWalls.end();

	while (iterator != end) {// !belongsToWall
		bool temp = (*iterator)->addMeasurement(pos); // TODO do not add to all
		belongsToWall = belongsToWall || temp;
		++iterator;
	}
	// std::cout << "belongsToWall " << belongsToWall << std::endl;
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

void Map::getVisualization(MapVisualization& vis) {
	for (std::list<WallSegment*>::const_iterator iter = mWalls.begin(), end = mWalls.end(); iter != end; ++iter) {
		vis.walls.push_back((*iter)->getVisualization());
	}
}