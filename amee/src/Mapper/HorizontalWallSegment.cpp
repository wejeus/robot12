#include "HorizontalWallSegment.h"
#include <cmath>

using namespace amee;

// Members:
// Map::Point mFrom, mTo; // mFrom is always left, mTo is always right
// unsigned int mNumberOfPoints;
// float mYAcc;
// Map::Point mDir;

std::ostream& amee::operator<< (std::ostream &out, const HorizontalWallSegment* seg) {
	out << seg->mFrom.x << ' ' << seg->mFrom.y << ' ' << seg->mTo.x << ' ' << seg->mTo.y << ' ' << seg->mNumberOfPoints << ' ' << seg->mYAcc;
	return out;
}

HorizontalWallSegment::HorizontalWallSegment(Map::Point pos) {
	mFrom = pos;
	mTo = pos;
	mNumberOfPoints = 1;
	mYAcc = pos.y;
}

HorizontalWallSegment::HorizontalWallSegment(std::istream& is) {
	is >> mFrom.x;
	is >> mFrom.y;
	is >> mTo.x;
	is >> mTo.y;
	is >> mNumberOfPoints;
	is >> mYAcc;
}

HorizontalWallSegment::~HorizontalWallSegment() {
}

WallVisualization HorizontalWallSegment::getVisualization(){
	WallVisualization vis;
	vis.startX = mFrom.x;
	vis.startY = mFrom.y;
	vis.endX = mTo.x;
	vis.endY = mTo.y;
	vis.type = 0;
	return vis;
}


bool HorizontalWallSegment::mergeWall(WallSegment* wall) {
	if (wall->getType() != HORIZONTAL) {
		return false;
	}
	HorizontalWallSegment* vWall = (HorizontalWallSegment*)wall;
	
	// first order both wall segements
	Map::Point lowerFrom = mFrom; // lower means lower values
	Map::Point lowerTo = mTo;	
	Map::Point upperFrom = vWall->mFrom; // upper means bigger values
	Map::Point upperTo = vWall->mTo;
	if (mFrom.x > vWall->mFrom.x) {
		lowerFrom = vWall->mFrom;
		lowerTo = vWall->mTo;
		upperTo = mTo;
		upperFrom = mFrom;
	} 

	// if they have a too large distance in y dimension we can't merge them
	if (fabs(upperTo.y - lowerTo.y) > ORTHOGONAL_MERGE_THRESHOLD) {
		return false;
	}

	// now check the distance in x.
	// check whether both walls are not overlapping and the parallel distance is to big
	if ((lowerTo.x < upperFrom.x) && (upperFrom.x - lowerTo.x > PARALLEL_MERGE_THRESHOLD)) {
		return false;
	}
	// if we reached this point, we want to merge the walls.

	// this is done by setting this wall to the new merged wall, the other wall can be deleted (not here).
	mFrom.x = std::min(lowerFrom.x, upperFrom.x);
	mTo.x = std::max(lowerTo.x, upperTo.x);

	mYAcc += vWall->mYAcc;
	mNumberOfPoints += vWall->mNumberOfPoints;
	mFrom.y = mYAcc / mNumberOfPoints;
	mTo.y = mFrom.y;

	return true;
}

bool HorizontalWallSegment::leftOf(HorizontalWallSegment* wall) {
	return mFrom.x < wall->mFrom.x;
}

bool HorizontalWallSegment::belowOf(HorizontalWallSegment* wall) {
	return mFrom.y < wall->mFrom.y;
}

bool HorizontalWallSegment::addMeasurement(const Map::Point& p) {
	// if (isInRange(p)) {
		++mNumberOfPoints;
		
		mFrom.x = mFrom.x < p.x ? mFrom.x : p.x;
		mTo.x = mTo.x > p.x ? mTo.x : p.x;

		mYAcc += p.y;
		mFrom.y = mYAcc / mNumberOfPoints;
		mTo.y = mFrom.y; 
		return true;
	// } 
	// return false;
}

int HorizontalWallSegment::getType() {
	return HORIZONTAL;
}

bool HorizontalWallSegment::isSmall() {
	float length = mTo.x - mFrom.x;
	// return (length/(float)mNumberOfPoints < SMALL_THRESHOLD) && !(length > SMALL_LENGTH);
	return length <= SMALL_LENGTH;
}

float HorizontalWallSegment::getX() {
	return (mFrom.x + mTo.x) / 2.0f;
}

float HorizontalWallSegment::getY() {
	return mFrom.y;
}

float HorizontalWallSegment::distanceTo(const Map::Point& pos) {
	if ((pos.x <= mTo.x) && (pos.x >= mFrom.x)) {
		return fabs(pos.y - mFrom.y);
	} else if (pos.x < mFrom.x) {
		return sqrt((pos.x - mFrom.x) * (pos.x - mFrom.x) + (pos.y - mFrom.y) * (pos.y - mFrom.y));
	}
	return sqrt((pos.x - mTo.x) * (pos.x - mTo.x) + (pos.y - mTo.y) * (pos.y - mTo.y));
}

// bool HorizontalWallSegment::isInRange(const Map::Point& pos) {
// 	bool inXRange = (mFrom.x - PARALLEL_TOLERANCE <= pos.x) && (pos.x <= mTo.x + PARALLEL_TOLERANCE);
// 	bool inYRange = (mFrom.y - WALL_THICKNESS / 2.0f - ORTHOGONAL_TOLERANCE <= pos.y) && (mFrom.y + WALL_THICKNESS / 2.0f + ORTHOGONAL_TOLERANCE >= pos.y);
// 	return inXRange && inYRange;
// }