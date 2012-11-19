#include "VerticalWallSegment.h"

using namespace amee;

// Members:
// Map::Point mFrom, mTo; // mFrom is always minimal, mTo is always maximal
// unsigned int mNumberOfPoints;
// float mXAcc;
// Map::Point mDir;

VerticalWallSegment::VerticalWallSegment(Map::Point pos) {
	mFrom = pos;
	mTo = pos;
	mNumberOfPoints = 1;
	mXAcc = pos.x;
}

VerticalWallSegment::~VerticalWallSegment() {
}

WallVisualization VerticalWallSegment::getVisualization(){
	WallVisualization vis;
	vis.startX = mFrom.x;
	vis.startY = mFrom.y;
	vis.endX = mTo.x;
	vis.endY = mTo.y;
	vis.type = VERTICAL;
	return vis;
}

bool VerticalWallSegment::mergeWall(WallSegment* wall) {
	if (wall->getType() != VERTICAL) {
		return false;
	}
	VerticalWallSegment* vWall = (VerticalWallSegment*)wall;
	
	// first order both wall segements
	Map::Point lowerFrom = mFrom; // lower means lower values
	Map::Point lowerTo = mTo;	
	Map::Point upperFrom = vWall->mFrom; // upper means bigger values
	Map::Point upperTo = vWall->mTo;
	if (mFrom.y > vWall->mFrom.y) {
		lowerFrom = vWall->mFrom;
		lowerTo = vWall->mTo;
		upperTo = mTo;
		upperFrom = mFrom;
	} 

	// if they have a too large distance in x dimension we can't merge them
	if (fabs(upperTo.x - lowerTo.x) > ORTHOGONAL_MERGE_THRESHOLD) {
		return false;
	}

	// now check the distance in y.
	// check whether both walls are not overlapping and the parallel distance is to big
	if ((lowerTo.y < upperFrom.y) && (upperFrom.y - lowerTo.y > PARALLEL_MERGE_THRESHOLD)) {
		return false;
	}
	// if we reached this point, we want to merge the walls.

	// this is done by setting this wall to the new merged wall, the other wall can be deleted (not here).
	mFrom = lowerFrom;
	mTo = upperTo;

	mXAcc += vWall->mXAcc;
	mNumberOfPoints += vWall->mNumberOfPoints;
	mFrom.x = mXAcc / mNumberOfPoints;
	mTo.x = mFrom.x;

	return true;

}

bool VerticalWallSegment::isSmall() {
	float length = mTo.y - mFrom.y;
	return (length/(float)mNumberOfPoints < SMALL_THRESHOLD) && !(length > SMALL_LENGTH);
}

bool VerticalWallSegment::addMeasurement(const Map::Point& p) {
	if (isInRange(p)) {
		++mNumberOfPoints;
		
		mFrom.y = mFrom.y < p.y ? mFrom.y : p.y;
		mTo.y = mTo.y > p.y ? mTo.y : p.y;

		mXAcc += p.x;
		mFrom.x = mXAcc / mNumberOfPoints;
		mTo.x = mFrom.x; 
		return true;
	} 
	return false;
}

int VerticalWallSegment::getType() {
	return VERTICAL;
}

float VerticalWallSegment::getX() {
	return mFrom.x;
}

float VerticalWallSegment::getY() {
	return (mFrom.y + mTo.y) / 2.0f;
}

float VerticalWallSegment::distanceTo(const Map::Point& pos) {
	if (pos.y < mFrom.y || pos.y > mTo.y) {
		return pos.x - mFrom.x;
	} else if (pos.y < mFrom.y) {
		return sqrt((pos.x - mFrom.x) * (pos.x - mFrom.x) + (pos.y - mFrom.y) + (pos.y - mFrom.y));
	}
	return sqrt((pos.x - mTo.x) * (pos.x - mTo.x) + (pos.y - mTo.y) + (pos.y - mTo.y));
}

bool VerticalWallSegment::isInRange(const Map::Point& pos) {
	bool inYRange = (mFrom.y - PARALLEL_TOLERANCE <= pos.y) && (pos.y <= mTo.y + PARALLEL_TOLERANCE);
	bool inXRange = (mFrom.x - WALL_THICKNESS / 2.0f - ORTHOGONAL_TOLERANCE <= pos.x) && (pos.x <= mFrom.x + WALL_THICKNESS / 2.0f + ORTHOGONAL_TOLERANCE);
	return inXRange && inYRange;
}