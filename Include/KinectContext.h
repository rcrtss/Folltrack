#ifndef KinectContext_h
#define KinectContext_h
#include <XnCppWrapper.h>

#define MAX_USERS 15
#define MAX_POSITIONS 150

class KinectContext
{
	public:
		KinectContext();
		~KinectContext();
		//OpenNI Getters
		xn::Context* GetXnContext();
		xn::UserGenerator* GetUserGenerator();
		xn::DepthGenerator* GetDepthGenerator();
		XnUInt16 GetTrackedUserID();
		//Setters
		void SetTrackedUserID(XnUInt16 ID);
		//Setup Methods
		XnBool fileExists(const char *fn);
		XnStatus StandardSetup();
		XnStatus InitGenerators(const char *fn);
		XnStatus RegisterCallbacks();
		XnStatus Run();
		//Lifecycle methods
		void RunAsync();
		void Loop();
		void Update();
		void Release();
		//Information methods
		int GetPosition(XnPoint3D & positionOut);
		void PrintCoM(int user);
		//statics
		static XnBool bNeedPose;
		static XnChar strPose[20];
		//LoopBool
		bool bStopLoop;

	private:
		int checkUserLeave(XnVector3D &position);
		int checkEqualaPositions(int maxIndex);

		xn::Context context;
		xn::ScriptNode scriptNode;
		xn::DepthGenerator depthGenerator;
		xn::UserGenerator userGenerator;
		
		XnStatus nRetVal;

		XnUInt16 nTrackedUser;
		XnUInt16 nUsers;
		XnUserID aUsers[ MAX_USERS ];
		int nPositionIndex;
		XnFloat aPositions[ MAX_POSITIONS ][3];
		XnVector3D position;
};
//Callback methods
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/);
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* /*pCookie*/);
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/);
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* /*pCookie*/);
#endif
