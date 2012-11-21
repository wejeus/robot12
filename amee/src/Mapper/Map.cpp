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
		
	// Localizes the robot based on the given pose and measurements in the map. If localizing is not successfull outPose = inPose
void Map::localize(const amee::Pose& inPose, const amee::Mapper::Measurement&[] measurements, amee::Pose& outPose) {
	//TODO
	std::vector<Map::Point> intersections; // see if constructor can take size argument
	intersections.resize(measurements.size());
	for (unsigned int i = 0; i < measurements.size(); ++i) {
		for (std::list<WallSegment*)::const_iterator iter = mWalls.begin(), end = mWalls.end(); iter != end; iter++) {
			if (b)
		}
	}
}

WallSegment* Map::findBestMatch(amee::Mapper::Measurement& m, Point& intersection) {
	std::list<WallSegment*>::const_iterator iterator = mWalls.begin(), end = mWalls.end();
	WallMatching bestMatch;
	bestMatch.t = 2.0f;
	bestMatch.wall = NULL;

	while (iterator != end) {// !belongsToWall
		WallSegment* wall = (*iterator);
		// TODO use new interface!
		float t;
		bool matches = wall->mapMeasurement(m.sensorPos, m.pos, intersection, t);
		if (matches && t < bestMatch.t) {
			bestMatch.t = t;
			bestMatch.wall = wall;
			m.pos = intersection;
			belongsToWall = true;
		}
		++iterator;
	}
	return bestMatch.wall;	
}

void Map::addMeasurement(amee::Mapper::Measurement& m, int newWallType) {
	Point intersection;
	WallSegment* match = findBestMatch(m, intersection);

	// std::cout << "belongsToWall " << belongsToWall << std::endl;
	if ((match == NULL) && (newWallType != WallSegment::NONE)) {
		if (newWallType == WallSegment::HORIZONTAL) {
			WallSegment* wall = new HorizontalWallSegment(m.pos);
			mWalls.push_back(wall);	
		} else {
			WallSegment* wall = new VerticalWallSegment(m.pos);
			mWalls.push_back(wall);
		}
	}
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