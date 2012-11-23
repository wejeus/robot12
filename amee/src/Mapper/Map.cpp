#include "Map.h"
#include <iostream>
#include "WallSegment.h"
#include "HorizontalWallSegment.h"
#include "VerticalWallSegment.h"
#include "Mapper.h"

using namespace amee;

Map::Map() {
}

Map::~Map() {
	for (std::list<VerticalWallSegment*>::const_iterator iter = mVerticalWalls.begin(), end = mVerticalWalls.end(); iter != end; ++iter) {
		delete *iter;	
	}
	for (std::list<HorizontalWallSegment*>::const_iterator iter = mHorizontalWalls.begin(), end = mHorizontalWalls.end(); iter != end; ++iter) {
		delete *iter;	
	}
}

void Map::insertHorizontal(HorizontalWallSegment* wall) {
	std::list<HorizontalWallSegment*>::iterator current = mHorizontalWalls.begin();
	std::list<HorizontalWallSegment*>::iterator end = mHorizontalWalls.end();
	bool inserted = false;
	while (current != end && !inserted) {
		if (!(*current)->belowOf(wall)) {
			mHorizontalWalls.insert(current, wall);
			inserted = true;
		}
		current++;
	}
	if (!inserted) {
		mHorizontalWalls.push_back(wall);
	}
}

void Map::insertVertical(VerticalWallSegment* wall) {
	std::list<VerticalWallSegment*>::iterator current = mVerticalWalls.begin();
	std::list<VerticalWallSegment*>::iterator end = mVerticalWalls.end();
	bool inserted = false;
	while (current != end && !inserted) {
		if (!(*current)->leftOf(wall)) {
			mVerticalWalls.insert(current, wall);
			inserted = true;
		}
		current++;
	}
	if (!inserted) {
		mVerticalWalls.push_back(wall);
	}
}

void Map::reduceNumWalls(const Point& pos, float distance) {
	std::list<HorizontalWallSegment*>::iterator prevHorizontal = mHorizontalWalls.begin();
	std::list<HorizontalWallSegment*>::iterator iterHor = mHorizontalWalls.begin();
	++iterHor;
	for (std::list<HorizontalWallSegment*>::iterator end = mHorizontalWalls.end(); iterHor != end; ++iterHor) {
		HorizontalWallSegment* wall = (*iterHor);
		float dist = wall->distanceTo(pos);
		if (dist < distance) {
			if ((*prevHorizontal)->mergeWall(wall)) {
				prevHorizontal = mHorizontalWalls.erase(iterHor);
				--prevHorizontal;
				iterHor = prevHorizontal;
				delete wall;
			}
		}// else if (wall->isSmall()) {
		// 	iterHor = mHorizontalWalls.erase(iterHor);
		// 	--iterHor; // decrease iterHor so that we don't skip a wall
		// 	delete wall;
		// }	
		prevHorizontal = iterHor;
	}

	std::list<VerticalWallSegment*>::iterator prevVertical = mVerticalWalls.begin();
	// std::list<WallSegment*>::iterator prevVertical = mWalls.begin(); 
	std::list<VerticalWallSegment*>::iterator iterVer = mVerticalWalls.begin();
	++iterVer;
	for (std::list<VerticalWallSegment*>::iterator end = mVerticalWalls.end(); iterVer != end; ++iterVer) {
		VerticalWallSegment* wall = (*iterVer);
		float dist = wall->distanceTo(pos);
		if (dist < distance) {
			if ((*prevVertical)->mergeWall(wall)) {
				prevVertical = mVerticalWalls.erase(iterVer);
				--prevVertical;
				iterVer = prevVertical;
				delete wall;
			}
		} //else if (wall->isSmall()) {
		// 	iterVer = mVerticalWalls.erase(iterVer);
		// 	--iterVer; // decrease iterHor so that we don't skip a wall
		// 	delete wall;
		// }	
		prevVertical = iterVer;
	}
}

bool Map::isPathCollisionFree(const Point& start, const Point& end, float radius) {
	
}
		
	// Localizes the robot based on the given pose and measurements in the map. If localizing is not successfull outPose = inPose
void Map::localize(const amee::Pose& inPose, const amee::Map::MeasurementSet& measurements, amee::Pose& outPose, bool left, bool right) {
	//TODO
	outPose = inPose;
	Point leftBack, leftFront, rightBack, rightFront;
	bool rightFrontWall = false;
	bool rightBackWall = false;
	bool leftFrontWall = false;
	bool leftBackWall = false;
	bool leftPoseEstimated = false;
	bool rightPoseEstimated = false;
	WallSegment* matchRF;
	WallSegment* matchRB;
	WallSegment* matchLF;
	WallSegment* matchLB;


	if (measurements.rightFront.valid) {
		matchRF = findBestMatch(measurements.rightFront, rightFront);
		rightFrontWall = matchRF != NULL;	
	}
	
	if (measurements.rightBack.valid) {
		matchRB = findBestMatch(measurements.rightBack, rightBack);
		rightBackWall = matchRB != NULL;	
	}
	
	if (measurements.leftFront.valid) {
		matchLF = findBestMatch(measurements.leftFront, leftFront);
		leftFrontWall = matchLF != NULL;	
	}
	
	if (measurements.leftBack.valid) {
		matchLB = findBestMatch(measurements.leftBack, leftBack);
		leftBackWall = matchLB != NULL;	
	}
	
	Pose rightEstimate = inPose;
	if (rightFrontWall && rightBackWall && (matchRB == matchRF) && right) { // measurements on the right side belong to the same wall
		
		rightPoseEstimated = true;

		float diffDist = measurements.rightFront.dist - measurements.rightBack.dist;
		float relativeTheta = atan(diffDist / Mapper::IR_BASE_RIGHT); 

		float wallTheta = getWallAngle(inPose, matchRB, false);

		rightEstimate.theta = wallTheta + relativeTheta;

		float meanDist = (measurements.rightFront.dist + measurements.rightBack.dist) / 2.0f;
	
 		if (matchRB->getType() == WallSegment::VERTICAL) {
			float sideOfWall = inPose.x <= matchRB->getX() ? -1.0f : 1.0f;
	
			// now we want to reset the x coordinate
			float normalDistToWall = cos(relativeTheta) * meanDist;
			float robotR = Mapper::ROBOT_RADIUS;
			float centerDistToWall = normalDistToWall + cos(relativeTheta) * robotR;
			rightEstimate.x = matchRB->getX() + sideOfWall * centerDistToWall;
		} else {
			float sideOfWall = inPose.y <= matchRB->getY() ? -1.0f : 1.0f;

			// now we want to reset the y coordinate
			float normalDistToWall = cos(relativeTheta) * meanDist;
			float centerDistToWall = normalDistToWall + cos(relativeTheta) * 0.12f;
			rightEstimate.y = matchRB->getY() + sideOfWall * centerDistToWall;
		}
	}

	Pose leftEstimate = inPose;
	if (leftFrontWall && leftBackWall && (matchLB == matchLF) && left) { // measurements on the left side belong to the same wall
		
		leftPoseEstimated = true;

		float diffDist = measurements.leftFront.dist - measurements.leftBack.dist;
		float relativeTheta = atan(diffDist / Mapper::IR_BASE_RIGHT); 

		float wallTheta = getWallAngle(inPose, matchLB, true);

		leftEstimate.theta = wallTheta - relativeTheta;

		float meanDist = (measurements.leftFront.dist + measurements.leftBack.dist) / 2.0f;
	
 		if (matchLB->getType() == WallSegment::VERTICAL) {
			float sideOfWall = inPose.x <= matchLB->getX() ? -1.0f : 1.0f;
	
			// now we want to reset the x coordinate
			float normalDistToWall = cos(relativeTheta) * meanDist;
			float robotR = Mapper::ROBOT_RADIUS;
			float centerDistToWall = normalDistToWall + cos(relativeTheta) * robotR;
			leftEstimate.x = matchLB->getX() + sideOfWall * centerDistToWall;
		} else {
			float sideOfWall = inPose.y <= matchLB->getY() ? -1.0f : 1.0f;

			// now we want to reset the y coordinate
			float normalDistToWall = cos(relativeTheta) * meanDist;
			float centerDistToWall = normalDistToWall + cos(relativeTheta) * 0.12f;
			leftEstimate.y = matchLB->getY() + sideOfWall * centerDistToWall;
		}
	}

	if (leftPoseEstimated && rightPoseEstimated) {
		outPose.x = 0.5f * (rightEstimate.x + leftEstimate.x);
		outPose.y = 0.5f * (rightEstimate.y + leftEstimate.y);
		outPose.theta = 0.5f * (rightEstimate.theta + leftEstimate.theta);
	} else if (leftPoseEstimated) {
		outPose = leftEstimate;
	} else if (rightPoseEstimated) {
		outPose = rightEstimate;
	}

    outPose.theta =  moveAngleToInterval02PI(outPose.theta);// move theta to [0,2PI]	
}

float Map::getWallAngle(const Pose& pose, WallSegment* wall, bool left) {
	// determine theta of the wall (orienation we are heading to)
	float wallTheta = 0.0f;
	if (!left) { // right wall
		if (wall->getType() == WallSegment::VERTICAL) {
		
	 		if (pose.x <= wall->getX()) {
				wallTheta = M_PI / 2.0f;
	// 				sideOfWall = -1.0f;
			} else {
				wallTheta = 3.0f / 2.0f * M_PI;
			}
		} else {
			if (pose.y <= wall->getY()) {
				wallTheta = M_PI;
				// sideOfWall = -1.0f;
			} else {
				wallTheta = 0.0f;
			}
		}	
	} else { // left wall
		if (wall->getType() == WallSegment::VERTICAL) {
	 		if (pose.x <= wall->getX()) {
				wallTheta = 3.0f / 2.0f * M_PI;
			} else {
				wallTheta = M_PI / 2.0f;
			}
		} else {
			if (pose.y <= wall->getY()) {
				wallTheta = 0.0f;
			} else {
				wallTheta = M_PI;
			}
		}	
	}
	
	return wallTheta;
}

float Map::moveAngleToInterval02PI(float theta) {
	float temp = theta - floor(theta / (2.0f * M_PI)) * 2.0f * M_PI;
	if (temp < 0.0f) {
		return (2.0f * M_PI - temp);
	} 
	return temp;
}

WallSegment* Map::findBestMatch(const Measurement& m, Point& intersection) {
	
	WallMatching bestMatch;
	bestMatch.t = 2.0f;
	bestMatch.wall = NULL;

	// TODO distinguish between new wall type (check only hor if Horizontal, only vert if Vert, both if none)
	for (std::list<HorizontalWallSegment*>::const_iterator iterator = mHorizontalWalls.begin(), end = mHorizontalWalls.end();iterator != end; iterator++) {
		HorizontalWallSegment* wall = (*iterator);
	
		float t;
		bool match = wall->belongsToWall(m.sensorPos, m.pos, intersection, t);
		if (match && t < bestMatch.t) {
			bestMatch.t = t;
			bestMatch.wall = wall;
		}
	}

	for (std::list<VerticalWallSegment*>::const_iterator iterator = mVerticalWalls.begin(), end = mVerticalWalls.end();iterator != end; iterator++) {
		VerticalWallSegment* wall = (*iterator);
	
		float t;
		bool match = wall->belongsToWall(m.sensorPos, m.pos, intersection, t);
		if (match && t < bestMatch.t) {
			bestMatch.t = t;
			bestMatch.wall = wall;
		}
	}

	return bestMatch.wall;	
}

void Map::addMeasurement(const Measurement& m, int newWallType) {
	Point intersection;
	WallSegment* match = findBestMatch(m, intersection);

	if (match != NULL) {
		match->mapMeasurement(m.sensorPos, m.pos, intersection);
	} else if(newWallType != WallSegment::NONE) {
		if (newWallType == WallSegment::HORIZONTAL) {
			HorizontalWallSegment* wall = new HorizontalWallSegment(m.pos);
			insertHorizontal(wall);	
		} else {
			VerticalWallSegment* wall = new VerticalWallSegment(m.pos);
			insertVertical(wall);
		}
	}
}

void Map::print() {
	std::cout << "Map has " << mHorizontalWalls.size() + mVerticalWalls.size() << " walls" << "\n";
}

void Map::getVisualization(MapVisualization& vis) {
	vis.walls.resize(mHorizontalWalls.size() + mVerticalWalls.size());
	int i = 0;
	for (std::list<HorizontalWallSegment*>::const_iterator iter = mHorizontalWalls.begin(), end = mHorizontalWalls.end(); iter != end; ++iter,++i) {
		vis.walls[i] = ((*iter)->getVisualization());
	}
	for (std::list<VerticalWallSegment*>::const_iterator iter = mVerticalWalls.begin(), end = mVerticalWalls.end(); iter != end; ++iter,++i) {
		vis.walls[i] = ((*iter)->getVisualization());
	}
}