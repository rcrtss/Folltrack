#ifndef ACTOIONBUMPERS_H_INCLUDED
#define ACTOIONBUMPERS_H_INCLUDED
#include "Aria.h"
//How to control bumpers activation using the bytes that getStallValue return
class ActionBumper : public ArAction {
	public:
	    ActionBumper(ArRobot *robot);
	    virtual ~ActionBumper(void) {
	    };
	    virtual ArActionDesired *fire(ArActionDesired currentDesired);
	    virtual void setRobot(ArRobot *robot);
	protected:
	    ArActionDesired pDesired;
	    ArRobot *pRobot;
	    int nCount;
};



#endif
