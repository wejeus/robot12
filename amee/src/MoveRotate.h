#ifndef MOVE_ROTATE_H
#define MOVE_ROTATE_H

#include "MovementStates.h"

namespace amee{
	class MoveRotate : public MovementStates{
	public:
		MoveRotate();
		~MoveRotate();

		virtual void init();
		virtual const bool& isRunning();
		virtual void doControl();
	private:
		bool running;
	}; //MoveRotate

}; //namespace amee

#endif //MOVE_ROTATE_H
