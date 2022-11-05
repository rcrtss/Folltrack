#include "ActionBumper.h"
//#include "Aria.h"
ActionBumper::ActionBumper(ArRobot *robot):ArAction("Sonara") {
    this->nCount = 0;
    this->pRobot = robot;
}

void ActionBumper::setRobot(ArRobot *robot) {
    ArAction::setRobot(robot);
}

ArActionDesired *ActionBumper::fire(ArActionDesired currentDesired) {
    nCount++;
    pDesired.reset();
	int reading=pRobot->getStallValue();
    if (nCount==3){
        nCount=0;
        if (reading==512){            
	#if LOG_BUMPER
            ArLog::log(ArLog::Normal, "LEFT BUMPER\n\n");
	#endif
            Aria::exit(1);
        }else if (reading==1024){
	#if LOG_BUMPER
            ArLog::log(ArLog::Normal, "MID BUMPER (BOTH BUMPERS)\n\n");
	#endif
	    Aria::exit(1);
        }else if (reading==2048){
	#if LOG_BUMPER
            ArLog::log(ArLog::Normal, "RIGHT BUMPER\n\n" );
	#endif
	    Aria::exit(1);
        }
	#if LOG_BUMPER
	else{
            ArLog::log(ArLog::Normal, "NO BUMPERS\n\n");
        }
	#endif
    }
    return &pDesired;
}


