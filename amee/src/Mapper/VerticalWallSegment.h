#ifndef VERTICAL_WALLSEGMENT_H
#define VERTICAL_WALLSEGMENT_H

#include "WallSegment.h"
#include "Map.h"

namespace amee {

class VerticalWallSegment : public WallSegment{

	public:
		VerticalWallSegment(Map::Point pos);
		virtual ~VerticalWallSegment();
		/* Adds the given point to this wall if it's close enough. If it was close enough it returns true, otherwise false.	*/
		virtual bool addMeasurement(const Map::Point& pos);
		virtual WallSegment::Orientation getOrientation() const;
		virtual WallVisualization getVisualization();
	private:
		Map::Point mFrom, mTo;
		unsigned int mNumberOfPoints;
		float mXAcc;

		bool isInRange(const Map::Point& pos);

	};
}
#endif