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
	std::list<WallSegment*>::iterator prevHorizontal = mWalls.begin();
	std::list<WallSegment*>::iterator prevVertical = mWalls.begin(); 
	std::list<WallSegment*>::iterator iter = mWalls.begin();
	++iter;
	for (std::list<WallSegment*>::iterator end = mWalls.end(); iter != end; ++iter) {
		WallSegment* wall = (*iter);
		float dist = wall->distanceTo(pos);
		if (dist < distance) {
			if (wall->getType() == WallSegment::HORIZONTAL) {
				if ((*prevHorizontal)->mergeWall(wall)) {
					prevHorizontal = mWalls.erase(iter);
					--prevHorizontal;
					iter = prevHorizontal;
					delete wall;
				}
			} else {
				if ((*prevVertical)->mergeWall(wall)) {
					prevVertical = mWalls.erase(iter);
					--prevVertical;
					iter = prevVertical;
					delete wall;
				}
			}
		} else if (wall->isSmall()) {
			iter = mWalls.erase(iter);
			--iter; // decrease iter so that we don't skip a wall
			delete wall;
		}	
	}
}

WallSegment* Map::addMeasurement(const Point& pos, int newWallType) {
	bool belongsToWall = false;
	std::list<WallSegment*>::const_iterator iterator = mWalls.begin(), end = mWalls.end();
	WallSegment* result = NULL;

	while (iterator != end) {// !belongsToWall
		WallSegment* wall = (*iterator);
		bool temp = wall->addMeasurement(pos); // TODO do not add to all
		belongsToWall = belongsToWall || temp;
		if (temp) { // TODO associate only to one wall!!!
			result = wall;
		}
		++iterator;
	}
	// std::cout << "belongsToWall " << belongsToWall << std::endl;
	if (!belongsToWall && (newWallType != WallSegment::NONE)) {
		if (newWallType == WallSegment::HORIZONTAL) {
			result = new HorizontalWallSegment(pos);
			mWalls.push_back(result);	
		} else {
			result = new VerticalWallSegment(pos);
			mWalls.push_back(result);
		}
	}
	//TODO tidy up
	return result;
}

void Map::print() {
	std::cout << "Map has " << mWalls.size() << " walls" << "\n";
}

void Map::getVisualization(MapVisualization& vis) {
	vis.walls.resize(mWalls.size());
	int i = 0;
	for (std::list<WallSegment*>::const_iterator iter = mWalls.begin(), end = mWalls.end(); iter != end; ++iter,++i) {
		vis.walls[i] = ((*iter)->getVisualization());
	}
}