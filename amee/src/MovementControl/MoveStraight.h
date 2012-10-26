#ifndef MOVE_STRAIGHT_H
#define MOVE_STRAIGHT_H

#include "MovementStates.h"

namespace amee{
	class MoveStraight : public MovementStates{
	public:
		MoveStop();
		~MoveStop();

		virtual void init();
		virtual const bool& isRunning();
		virtual void doControl();
	private:
		bool running;
	}; //MoveStraight

}; //namespace amee

#endif //MOVE_STRAIGHT_H
