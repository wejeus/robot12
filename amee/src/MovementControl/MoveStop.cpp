#include "MoveStop.h"
#include "amee/Velocity.h"

namespace amee{


	MoveStop::MoveStop(ros::Publisher & pub){
		mPub = pub;
	}

	//destructor
	MoveStop::~MoveStop(){}

	void MoveStop::init(const SensorData &data){}

	bool MoveStop::isRunning() const {return true;}

	/**
	 * Publishes a 
	 **/
	void MoveStop::doControl(const SensorData &data){
		Velocity v; v.right = 0.0f; v.left = 0.0f;
		mPub.publish(v);
	}
}; // namespace amee
