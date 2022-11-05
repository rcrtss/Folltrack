//logging variables
#define LOG_KINECT true
#define LOG_FORCES false
#define LOG_BUMPERS false

//includes
//Kinect includes
#include <XnCppWrapper.h>
#include <KinectContext.h>
///Robot control includes
#include <Aria.h>
#include <ActionFollow.h>
#include <ActionBumper.h>
//end includes

using namespace std;

//Global variables used in a program
KinectContext g_KinectContext;

Params stdParams;

#define CHECK_RC(nRetVal, what)							\
	if (nRetVal != XN_STATUS_OK)						\
	{									\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));	\
		return nRetVal;							\
	}									\

//Defining parameters used by the robot
void initParams()
{
	stdParams.nStopDist = 1500; // distance between the robot and a target when the robot stops
	stdParams.nAttractionScale = 10000; // how strong the robot is attracted by a target
	stdParams.nRepulsiveScale = 200000000; //how strong the robot is repulsed by an obstacle
	stdParams.nLaserDivisionAmount = 10; // to how many parts laser detection is devided
	stdParams.nLaserWidth = 200; // how many degrees around laser observes (shouldn't be changed)
	stdParams.nTargetAreaRightBound = -10; // right border (angles) of a sphere where detection range is < nStopDist not to detect target as an obstacle
	stdParams.nTargetAreaLeftBound = 10; // left border (angles) of a sphere where detection range is < nStopDist not to detect target as an obstacle
	stdParams.nLaserStep= stdParams.nLaserWidth / stdParams.nLaserDivisionAmount;
	stdParams.nKinectAngle = 0;
}

int main(int argc, char** argv) {

    initParams();

    if (argc > 0)
    {
	for(int i=1; i<argc; i++)
	{
	    if(std::string(argv[i]) == "-d" || std::string(argv[i]) == "--stopdistance")
	    {
		i++;
		stdParams.nStopDist = atoi(argv[i]);
	    }
	    else if(std::string(argv[i]) == "-a" || std::string(argv[i]) == "--attractionforce")
	    {
		i++;
		stdParams.nAttractionScale = atoi(argv[i]);
	    }
	    else if(std::string(argv[i]) == "-r" || std::string(argv[i]) == "--repulsiveforce")
	    {
		i++;
		stdParams.nRepulsiveScale = atoi(argv[i]);
	    } 
	    else if(std::string(argv[i]) == "-l" || std::string(argv[i]) == "--laserdivisionamount")
	    {
		i++;
		stdParams.nLaserDivisionAmount = atoi(argv[i]);
	    }
	    else
	    {
	        printf("Option %s is not allowed", argv[i]);
	    }
	}
    }

    //initiate Kinect context
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = g_KinectContext.StandardSetup();
    CHECK_RC(nRetVal, "Kinect context setup");
    //end initiate kinect
    //start kinect
    g_KinectContext.RunAsync();

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot robot;
    ArRobotConnector robotConnector(&parser, &robot);
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
    if (!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "::::::: Could not connect to the robot.");
        if (parser.checkHelpAndWarnUnparsed()) {
            // -help not given
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
    }
    robot.runAsync(true);
    ArLog::log(ArLog::Normal, "::::::: Connected to robot.");
    laserConnector.connectLasers();
    ArTime timer0;
    timer0.setToNow();
    //Instance of actions
    ActionFollow go(timer0);
    go.setKinectContext(&g_KinectContext);
    go.setParams(&stdParams);
    ActionBumper bumpers(&robot);
    //ArActionBumpers bumpers("bumpers",100,2000,2000,false);//name, backoff speed, backoff time, turn time
    //Add actions to robot
    robot.addAction(&bumpers,70);
    robot.addAction(&go, 50);
    robot.enableMotors();
//    robot.run(true);
    printf("stopdistance set to: %d", stdParams.nStopDist);
    while(!xnOSWasKeyboardHit()){}
    Aria::exit(0);
}
