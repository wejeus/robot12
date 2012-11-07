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
	vis.type = 1;
	return vis;
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

bool VerticalWallSegment::isInRange(const Map::Point& pos) {
	bool inYRange = (mFrom.y - PARALLEL_TOLERANCE <= pos.y) && (pos.y <= mTo.y + PARALLEL_TOLERANCE);
	bool inXRange = (mFrom.x - WALL_THICKNESS / 2.0f - ORTHOGONAL_TOLERANCE <= pos.x) && (pos.x <= mFrom.x + WALL_THICKNESS / 2.0f + ORTHOGONAL_TOLERANCE);
	return inXRange && inYRange;
}

WallSegment::Orientation VerticalWallSegment::getOrientation() const {
	return Vertical;
}