#ifndef MOVE_STOP_H
#define MOVE_STOP_H

#include "MovementStates.h"

namespace amee{
	class MoveStop : public MovementStates{
	public:
		MoveStop();
		~MoveStop();

		virtual void init();
		virtual const bool& isRunning();
		virtual void doControl();
	private:
		bool running;
	}; //MoveStop

}; //namespace amee

#endif //MOVE_STOP_H
