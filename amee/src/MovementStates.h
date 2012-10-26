#ifndef MOVEMENT_STATES_H
#define MOVEMENT_STATES_H


namespace amee{

	enum STATE{ MOVE_STOP, MOVE_STRAIGHT, MOVE_ROTATE, MOVE_COORDINATE, MOVE_FOLLOW_WALL, MOVE_LOST};

	class MovementStates{
	public:
		//Constructors & destructor
	        MovementStates(){};
		virtual ~MovementStates(){};

		virtual void init() = 0;
		virtual const bool& isRunning() const = 0;
		virtual void doControl() const = 0;
		
//	private:
//		bool running;
	}; //MovementStates	

}; //namespace amee

#endif
