#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H


namespace amee{

//	enum STATE{ MOVE_STOP, MOVE_STRAIGHT, MOVE_ROTATE, MOVE_COORDINATE, MOVE_FOLLOW_WALL, MOVE_LOST};

	struct StrategyData {
		float lol;
	};

	class StrategyState{
	public:
		//Constructors & destructor
		StrategyState(){};
		virtual ~StrategyState(){};


		virtual void init(const StrategyData& data) = 0;
		virtual bool isRunning() const = 0;
		virtual void doControl(const StrategyData& data) = 0;
		
	}; //StrategyState

}; //namespace amee

#endif
