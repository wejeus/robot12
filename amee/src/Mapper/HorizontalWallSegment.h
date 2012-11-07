#ifndef HORIZONTAL_WALLSEGMENT_H
#define HORIZONTAL_WALLSEGMENT_H

#include "WallSegment.h"
#include "Map.h"

namespace amee {

class HorizontalWallSegment : public WallSegment{

	public:
		enum Orientation {Horizontal, Vertical};
		HorizontalWallSegment(Map::Point pos);
		virtual ~HorizontalWallSegment();
		/* Adds the given point to this wall if it's close enough. If it was close enough it returns true, otherwise false.	*/
		virtual bool addMeasurement(const Map::Point& pos);
		virtual Orientation getOrientation() const;
	private:
		Map::Point mFrom, mTo;
		unsigned int mNumberOfPoints;
		float mYAcc;

		bool isInRange(const Map::Point& pos);

	};
}
#endif