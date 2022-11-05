#ifndef ActionFollow_h
#define ActionFollow_h
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <cmath>
#include <Aria.h>
#include <KinectContext.h>

using namespace std; 

struct Params
{
	int nStopDist;
	int nLaserDivisionAmount;
	int nLaserWidth;	
	int nTargetAreaRightBound;
	int nTargetAreaLeftBound;
	int nAttractionScale;
	int nRepulsiveScale;
	double nLaserStep;
	double nKinectAngle;
};

class ActionFollow : public ArAction {
	public:
	    ActionFollow(ArTime &timer0);
	    virtual ~ActionFollow(void) {
	    };
	    virtual ArActionDesired *fire(ArActionDesired currentDesired);
	    virtual void setRobot(ArRobot *robot);
	    void setParams(Params *p);
	    void setKinectContext(KinectContext *context);
	protected:
	    Params *pParams;
	    KinectContext *pKinectContext;
	    ArRobot *pRobot;
	    ArRangeDevice *pLaser;
	    ArActionDesired pDesired;
	    vector<double> distances;
	    vector<double> angles;
	    vector<double> forces;
	    ofstream outfile;
	    ArTime timer0;
	    ArTime timer;
};
#endif
