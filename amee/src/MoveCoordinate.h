#ifndef MOVE_COORDINATE_H
#define MOVE_COORDINATE_H

#include "MovementStates.h"

namespace amee{
	class MoveCoordinate : public MovementStates{
	public:
		MoveCoordinate();
		~MoveCoordinate();

		virtual void init();
		virtual const bool& isRunning();
		virtual void doControl();
	private:
		bool running;
	}; //MoveCoordinate

}; //namespace amee

#endif //MOVE_COORDINATE_H
