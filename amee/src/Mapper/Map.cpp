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
	outPose = inPose;
	Point leftBack, leftFront, rightBack, rightFront;
	bool rightFrontWall = false;
	bool rightBackWall = false;
	bool leftFrontWall = false;
	bool leftBackWall = false;
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
	
	if (rightFrontWall && rightBackWall && (matchRB == matchRF)) { // measurements on the right side belong to the same wall
		Pose rightEstimate = inPose;
		// float angleMeasurements = getAngle(rightFront - rightBack);

		float diffDist = measurements.rightFront.dist - measurements.rightBack.dist;
		float relativeTheta = atan(diffDist / Mapper::IR_BASE_RIGHT); 

		float wallTheta = getWallAngle(inPose, matchRB);

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

		// Point correctedSensorFront = rightFront + (measurements.rightFront.sensorPos - rightFront).normalized() * measurements.rightFront.dist;
	 //    Point correctedSensorBack = rightBack + (measurements.rightBack.sensorPos - rightBack).normalized() * measurements.rightBack.dist;

	 //    Point frontRelativeToPose = measurements.rightFront.sensorRelativePos;
	 //    frontRelativeToPose.rotate(rightEstimate.theta);
	 //    Point frontBasedEstimation = correctedSensorFront - frontRelativeToPose;

	 //    Point backRelativeToPose = measurements.rightBack.sensorRelativePos;
	 //    backRelativeToPose.rotate(rightEstimate.theta);
	 //    Point backBasedEstimation = correctedSensorBack - backRelativeToPose;

	 //    rightEstimate.x = 0.5f * frontBasedEstimation.x + 0.5 * backBasedEstimation.x; // we trust both estimations the same
	 //    rightEstimate.y = 0.5f * frontBasedEstimation.y + 0.5 * backBasedEstimation.y;

	    outPose.x = rightEstimate.x;
	    outPose.y = rightEstimate.y;
	    outPose.theta = rightEstimate.theta;
	    outPose.theta =  moveAngleToInterval02PI(outPose.theta);// move theta to [0,2PI]
	    // std::cout << "est. pose in map: x:" << outPose.x << " y:" << outPose.y << " theta: " << outPose.theta << std::endl; 
			
	// 		// determine relative theta to the wall
	// 		float diffDist = mDistances.rightFront - mDistances.rightBack;
	// 		float meanDist = (mDistances.rightFront + mDistances.rightBack) / 2.0f;
	// 		float relativeTheta = atan(diffDist / IR_BASE_RIGHT);
	// 		// std::cout << "Old pose: x:" << mPose.x << " y:" << mPose.y << " theta: " << mPose.theta << std::endl;
	// 		// determine theta of the wall (orienation we are heading to)
	

	// 			// now reset theta accordingly
	// 			mPose.theta = wallTheta + relativeTheta;

	// 			// now we want to reset the y coordinate
	// 			float normalDistToWall = cos(relativeTheta) * meanDist;
	// 			float centerDistToWall = normalDistToWall + cos(relativeTheta) * 0.12f;
	// 			mPose.y = wall->getY() + sideOfWall * centerDistToWall;
	// 		}
	}
	
	
	
	// TODO left estimation

	
}

float Map::getWallAngle(const Pose& pose, WallSegment* wall) {
	// determine theta of the wall (orienation we are heading to)
	float wallTheta = 0.0f;
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
	return wallTheta;
}

float Map::moveAngleToInterval02PI(float theta) {
	float temp = theta - floor(theta / (2.0f * M_PI)) * 2.0f * M_PI;
	if (temp < 0.0f) {
		return (2.0f * M_PI - temp);
	} 
	return temp;
}

float Map::getAngle(const Point& dir) {
	if (dir.x == 0.0f) {
		if (dir.y > 0.0f) {
			return M_PI / 2.0f;
		}
		return 3.0f / 2.0f * M_PI;
	}
	if (dir.y == 0.0f) {
		if (dir.x > 0.0f) {
			return 0.0f;
		}
		return M_PI;
	}
	return atan(dir.y / dir.x);
}

WallSegment* Map::findBestMatch(const Measurement& m, Point& intersection) {
	
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