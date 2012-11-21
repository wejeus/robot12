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
void Map::localize(const amee::Pose& inPose, const amee::Map::MeasurementSet& measurements, amee::Pose& outPose) {
	//TODO
	Point leftBack, leftFront, rightBack, rightFront;

	WallSegment* bestMatch = findBestMatch(measurements.rightFront, rightFront);
	bool rightFrontWall = bestMatch != NULL;
	bestMatch = findBestMatch(measurements.rightBack, rightBack);
	bool rightBackWall = bestMatch != NULL;
	bestMatch = findBestMatch(measurements.leftFront, leftFront);
	bool leftFrontWall = bestMatch != NULL;
	bestMatch = findBestMatch(measurements.leftBack, leftBack);
	bool leftBackWall = bestMatch != NULL;
	
	// TODO

	
}

WallSegment* Map::findBestMatch(amee::Mapper::Measurement& m, Point& intersection) {
	
	WallMatching bestMatch;
	bestMatch.t = 2.0f;
	bestMatch.wall = NULL;

	for (std::list<WallSegment*>::const_iterator iterator = mWalls.begin(), end = mWalls.end();iterator != end; iterator++) {
		WallSegment* wall = (*iterator);
	
		float t;
		bool match = wall->belongsToWall(m.sensorPos, m.pos, intersection, t);
		if (match && t < bestMatch.t) {
			bestMatch.t = t;
			bestMatch.wall = wall;
			m.pos = intersection;
		}
	}
	return bestMatch.wall;	
}

void Map::addMeasurement(amee::Mapper::Measurement& m, int newWallType) {
	Point intersection;
	WallSegment* match = findBestMatch(m, intersection);

	if (match != NULL) {
		match->mapMeasurement(m.sensorPos, m.pos, intersection);
	} else if(newWallType != WallSegment::NONE)) {
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