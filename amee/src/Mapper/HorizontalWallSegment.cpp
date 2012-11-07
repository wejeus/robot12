#include "HorizontalWallSegment.h"

using namespace amee;

// Members:
// Map::Point mFrom, mTo; // mFrom is always left, mTo is always right
// unsigned int mNumberOfPoints;
// float mYAcc;
// Map::Point mDir;

HorizontalWallSegment::HorizontalWallSegment(Map::Point pos) {
	mFrom = pos;
	mTo = pos;
	mNumberOfPoints = 1;
	mYAcc = pos.y;
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

bool HorizontalWallSegment::addMeasurement(const Map::Point& p) {
	if (isInRange(p)) {
		++mNumberOfPoints;
		
		mFrom.x = mFrom.x < p.x ? mFrom.x : p.x;
		mTo.x = mTo.x > p.x ? mTo.x : p.x;

		mYAcc += p.y;
		mFrom.y = mYAcc / mNumberOfPoints;
		mTo.y = mFrom.y; 
		return true;
	} 
	return false;
}

bool HorizontalWallSegment::isInRange(const Map::Point& pos) {
	bool inXRange = (mFrom.x - PARALLEL_TOLERANCE <= pos.x) && (pos.x <= mTo.x + PARALLEL_TOLERANCE);
	bool inYRange = (mFrom.y - WALL_THICKNESS / 2.0f - ORTHOGONAL_TOLERANCE <= pos.y) && (mFrom.y + WALL_THICKNESS / 2.0f + ORTHOGONAL_TOLERANCE >= pos.y);
	return inXRange && inYRange;
}

WallSegment::Orientation HorizontalWallSegment::getOrientation() const{
	return Horizontal;
}