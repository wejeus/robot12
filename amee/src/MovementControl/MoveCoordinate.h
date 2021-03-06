#ifndef MOVE_COORDINATE_H
#define MOVE_COORDINATE_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate;
	class MoveStraight;

	class MoveCoordinate : public MovementState{

	public:
		MoveCoordinate(ros::Publisher &pub, ros::Publisher& movement_event_pub);
		~MoveCoordinate();

		virtual void init(const SensorData &data);
		void init(const SensorData &data, const float &x, const float &y);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);

	private:
		// static const float DISTANCE_THRESHHOLD = 0.01f; //in meters

		bool mRunning;
		bool mRotationDone;

		float mDistance; // the destination position

		ros::Publisher mPub;
		ros::Publisher mMovementEventPub;

		// allows us to rotate by calling its doControl after we initialized it as long as we want to rotate
		amee::MoveRotate *mRotater;
		// allows us to move straight
		amee::MoveStraight *mStraightMove;

		// bool distReached() const;
		// float getRotationAngle(const float[2], const float[2]) const;
		float checkDirection(const float) const;
		float euclidDist(const float[2], const float[2]) const;
		float norm(const float[2]) const;
		bool wallInFront(const SensorData& data);
		
	}; //MoveCoordinate

}; //namespace amee

#endif //MOVE_COORDINATE_H
