#ifndef MOVE_FOLLOW_WALL_H
#define MOVE_FOLLOW_WALL_H

#include "MovementStates.h"

namespace amee{
	class MoveFollowWall : public MovementStates{
	public:
		MoveFollowWall();
		~MoveFollowWall();

		virtual void init();
		virtual const bool& isRunning();
		virtual void doControl();
	private:
		bool running;
	}; //MoveFollowWall

}; //namespace amee

#endif //MOVE_FOLLOW_WALL_H
