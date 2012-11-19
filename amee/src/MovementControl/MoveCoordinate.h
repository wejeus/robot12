#ifndef MOVE_COORDINATE_H
#define MOVE_COORDINATE_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate;
	class MoveStraight;

	class MoveCoordinate : public MovementState{

	public:
		MoveCoordinate(ros::Publisher &pub);
		~MoveCoordinate();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);

	private:
		static const float DISTANCE_THRESHHOLD = 0.01f; //in meters

		bool mRunning;
		bool mFirstRun;
		bool mRotationDone;

		float mX, mY, mAngle, mDistance; // the destination position
		float mCurX, mCurY, mCurAngle; // the current position

		ros::Publisher mPub;

		amee::SensorData mSensorData;

		// allows us to rotate by calling its doControl after we initialized it as long as we want to rotate
		amee::MoveRotate *mRotater;
		// allows us to move straight
		amee::MoveStraight *mStraightMove;

		bool distReached() const;
		float getRotationAngle(const float[2], const float[2]) const;
		float checkDirection(const float) const;
		float euclidDist(const float[2], const float[2]) const;
		float norm(const float[2]) const;
		
	}; //MoveCoordinate

}; //namespace amee

#endif //MOVE_COORDINATE_H
