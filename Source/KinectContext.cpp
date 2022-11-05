#include <iostream>
#include <thread>
#include <XnCppWrapper.h>
#include <KinectContext.h>

#define LOG_KINECT true

XnBool KinectContext::bNeedPose = false;
XnChar KinectContext::strPose[20] = "";

KinectContext::KinectContext()
	{}

KinectContext::~KinectContext()
{
	Release();
}

XnBool KinectContext::fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

#define STD_XML_PATH "SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)							\
	if (nRetVal != XN_STATUS_OK)						\
	{									\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));	\
		return nRetVal;							\
	}	
/* 
* Setup to initiate all generators and register callbacks
* with standard xml file path
*/
XnStatus KinectContext::StandardSetup()
{
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = InitGenerators(STD_XML_PATH);
	CHECK_RC(nRetVal, "Initiating generators");

	nRetVal = RegisterCallbacks();
	CHECK_RC(nRetVal, "registering callbacks");

	userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	
	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	
	printf("logging: kinect=%d\n", LOG_KINECT);

	return nRetVal;
}

/*
* Init Scriptnode, UserGenerator and DepthGenerator from file @param fn
*/
XnStatus KinectContext::InitGenerators(const char *fn)
{
	if(!fileExists(fn)) 
	{
		printf("File %s does not exist", fn);
		return 1;
	}

	xn::EnumerationErrors errors;

	nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);
	if(nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return nRetVal;
	}
	else if(nRetVal != XN_STATUS_OK)
	{
		return nRetVal;
	}
	
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
	CHECK_RC(nRetVal, "No depth generator");

	nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
	if(nRetVal != XN_STATUS_OK)
	{
		nRetVal = userGenerator.Create(context);
		CHECK_RC(nRetVal, "Create user generator");
	}

	return nRetVal;
}

/*
* Register callback functions for:
*	- New user
* 	- Lost user
*	- Calibration start
*	- Calibration complete
*	- Pose detection (if necessary)
*/
XnStatus KinectContext::RegisterCallbacks()
{
	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	
	nRetVal = userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, this, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	if (userGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		printf("pose is required\n");
		bNeedPose = TRUE;
		if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		nRetVal = userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, this, hPoseDetected);
		CHECK_RC(nRetVal, "Register to Pose Detected");
		userGenerator.GetSkeletonCap().GetCalibrationPose(strPose);
	}
	
	return nRetVal;
}

struct sLoop
{
	void operator() (KinectContext* & context) const
	{
		while(!context->bStopLoop)
		{
			context->Update();
		}
	}
};

void KinectContext::RunAsync()
{
	bStopLoop = false;
	sLoop f;
	std::thread t(f, this);
	t.detach();
}

void KinectContext::Update()
{
	context.WaitOneUpdateAll(userGenerator);
	nUsers = userGenerator.GetNumberOfUsers();
	nUsers = MAX_USERS; 
	userGenerator.GetUsers(aUsers, nUsers);

	//printf("tracking %d: %s\n", aUsers[nTrackedUser - 1], userGenerator.GetSkeletonCap().IsTracking(nTrackedUser) ? "true" : "false"); 
	if(nTrackedUser != 0 && userGenerator.GetSkeletonCap().IsTracking(nTrackedUser) != FALSE)
	{
		//printf("position update");
		XnSkeletonJointPosition jointPosition;
		userGenerator.GetSkeletonCap().GetSkeletonJointPosition(nTrackedUser, XN_SKEL_TORSO, jointPosition);
		position = jointPosition.position;
	}

	//if(checkUserLeave(position) == -1) nTrackedUser = 0;
}

/*
* Print Center of Mass of specified user or first user
*/
void KinectContext::PrintCoM(int user = 1)
{
	if(nUsers >= user)
	{
		XnUInt32 epochTime = 0;
		xnOSGetEpochTime(&epochTime);
		XnPoint3D _CoM;
		userGenerator.GetCoM(user, _CoM);
		printf("%d: user %d: center of mass at %6.2f,%6.2f,%6.2f\n",epochTime, user, 
				_CoM.X,
				_CoM.Y,
				_CoM.Z);
	}
}

int KinectContext::GetPosition(XnVector3D &positionOut)
{
	if(nTrackedUser == 0) return -1;
	if(position.X == 0 && position.Y == 0 && position.Z == 0)
	       return -1;
	if(checkUserLeave(position) == -1) 
		return -1;
	positionOut = position;
	return 0;	
}

int KinectContext::checkUserLeave(XnVector3D &position)
{
	aPositions[nPositionIndex][0] = position.X;
	aPositions[nPositionIndex][1] = position.Y;
	aPositions[nPositionIndex][2] = position.Z;

	int res = checkEqualaPositions(MAX_POSITIONS);
	if(nPositionIndex < MAX_POSITIONS - 1) nPositionIndex++; 
	else nPositionIndex = 0;
	//printf("res: %d", res);
	//printf(res != 0 ? "user present" : "user gone");
	return res;
}

int KinectContext::checkEqualaPositions(int maxIndex)
{
	for(int i=1; i< maxIndex; i++)
	{
		if(!(aPositions[i][0] == aPositions[0][0]
		   && aPositions[i][1] == aPositions[0][1]
		   && aPositions[i][2] == aPositions[0][2]))
			return 0;
	}
	return -1;
}

void KinectContext::Release()
{
	bStopLoop = true;
	scriptNode.Release();
	depthGenerator.Release();
	userGenerator.Release();
	context.Release();
}

xn::Context* KinectContext::GetXnContext(){ return &context; }
xn::UserGenerator* KinectContext::GetUserGenerator(){ return &userGenerator; }
xn::DepthGenerator* KinectContext::GetDepthGenerator(){ return &depthGenerator; }

XnUInt16 KinectContext::GetTrackedUserID(){ return nTrackedUser; }
void KinectContext::SetTrackedUserID(XnUInt16 ID)
{ 
	nTrackedUser = ID;
	//#if LOG_KINECT
	printf("Tracked user: %d\n", nTrackedUser); 
	//#endif
}

/*
* timout for restarting pose detection again
*/
struct RestartPoseDetection
{
	void operator() (xn::UserGenerator* & generator, XnUInt16 nId) const
	{
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			#if LOG_KINECT
			printf("pose detection restart");
			#endif
			generator->GetPoseDetectionCap().StartPoseDetection(KinectContext::strPose, nId);
	}
};
/*
* New user found, start pose detection if necessary, otherwise start calibration
*/
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	#if LOG_KINECT
	printf("%d New User %d\n", epochTime, nId);
	#endif
	// New user found
	if (KinectContext::bNeedPose)
	{
		generator.GetPoseDetectionCap().StartPoseDetection(KinectContext::strPose, nId);
	}
	else
	{
		generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	#if LOG_KINECT
	printf("%d Lost user %d\n", epochTime, nId);	
	#endif

	XnUInt16 nTrackedUser = reinterpret_cast<KinectContext*>(pCookie)->GetTrackedUserID();
	if(nTrackedUser == nId) reinterpret_cast<KinectContext*>(pCookie)->SetTrackedUserID(0);
}
/*
* Pose detected, stop pose detection and start calibration for user
*/
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	#if LOG_KINECT
	printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
	#endif
	capability.StopPoseDetection(nId);
	KinectContext* kinectContext = reinterpret_cast<KinectContext*>(pCookie);

	if(kinectContext->GetTrackedUserID() == nId) 
	{
		kinectContext->SetTrackedUserID(0);
		RestartPoseDetection f;
		std::thread t(f, kinectContext->GetUserGenerator(), nId);
		t.detach();
	}
	else kinectContext->GetUserGenerator()->GetSkeletonCap().RequestCalibration(nId, TRUE);


	//kinectContext->GetUserGenerator()->GetSkeletonCap().RequestCalibration(nId, TRUE);
}
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	#if LOG_KINECT
	printf("%d Calibration started for user %d\n", epochTime, nId);	
	#endif
}

/*
* Calibration complete, start tracking
* if aborted restart pose detection if necessary or restart calibration
*/
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
	KinectContext* kinectContext = reinterpret_cast<KinectContext*>(pCookie);
	xn::UserGenerator* generator = kinectContext->GetUserGenerator();
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		// Calibration succeeded
		#if LOG_KINECT
		printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);		
		#endif

		XnStatus nRetVal = generator->GetSkeletonCap().StartTracking(nId);
		if(nRetVal != XN_STATUS_OK) printf("%s failed: %s\n", "start tracking", xnGetStatusString(nRetVal));
		if(kinectContext->GetTrackedUserID() == 0)
		{
			kinectContext->SetTrackedUserID(nId);
			RestartPoseDetection f;
			std::thread t(f, generator, nId);
			t.detach();
		}
	}
	else
	{
		// Calibration failed
		#if LOG_KINECT
		printf("%d Calibration failed for user %d\n", epochTime, nId);
		#endif
		if(kinectContext->GetTrackedUserID() == nId) kinectContext->SetTrackedUserID(0);

		if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
		{
		    printf("Manual abort occured, stop attempting to calibrate!");
		    return;
		}
		if (KinectContext::bNeedPose)
		{
			generator->GetPoseDetectionCap().StartPoseDetection(KinectContext::strPose, nId);
		}
		else
		{
			generator->GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}

}
