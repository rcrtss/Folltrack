#include <ActionFollow.h>
#include <ctime>

ActionFollow::ActionFollow(ArTime &timer0) : ArAction("Laser") {
    this->pLaser = NULL;
    this->timer0 = timer0;
    pParams->nRepulsiveScale = 200000000;
    //Save distance to target in a file
    outfile.open("DistanceData.txt", std::ofstream::out | std::ofstream::trunc);
    //outfile << "time" << "||" << "pRobot->getTh()" << "||" << "nFinalAngle" << "||" << "nRepulsiveForceX" << "||" << "nRepulsiveForceY" << std::endl;
    outfile << "Distance robot-person" << std::endl;
    outfile.close();
    //Save distance to obstacle in a file
    outfile.open("ObstacleData.txt", std::ofstream::out | std::ofstream::trunc);
    //outfile << "time" << "||" << "pRobot->getTh()" << "||" << "nFinalAngle" << "||" << "nRepulsiveForceX" << "||" << "nRepulsiveForceY" << std::endl;
    outfile << "Distance robot-obstacles" << std::endl;
    outfile.close();


}

void ActionFollow::setRobot(ArRobot *robot) {
    ArAction::setRobot(robot);
    pLaser = robot->findLaser(1);
    pRobot = robot;
}

void ActionFollow::setParams(Params *p)
{
    pParams = p;
    pParams->nLaserStep = pParams->nLaserWidth / pParams->nLaserDivisionAmount;
}

void ActionFollow::setKinectContext(KinectContext *context)
{
    pKinectContext = context;
}

ArActionDesired *ActionFollow::fire(ArActionDesired currentDesired) {
    pDesired.reset();

   //get readings from laser and save the closest one to a file for a graph
   int closest = 20000;
   for (int i = 0; i < pParams->nLaserDivisionAmount; i++) {
        double buff;
        distances.push_back(pLaser->currentReadingPolar(-100 + i * pParams->nLaserStep, (-100 + pParams->nLaserStep) + i * pParams->nLaserStep - 1, &buff));
        angles.push_back(buff);
	if(distances[i] < closest) closest = distances[i];
    }    
    //write closest distance to file for a graph
    outfile.open("ObstacleData.txt", std::ios_base::app);
    outfile << closest << std::endl;
    outfile.close();

    //set range where target is, to not detect target as obstacle
    for (int i = 0; i < pParams->nLaserDivisionAmount; i++) {
	if((-100 + pParams->nLaserStep*i < pParams->nTargetAreaRightBound + pParams->nKinectAngle) || (-100 + pParams->nLaserStep*i > pParams->nTargetAreaLeftBound + pParams->nKinectAngle)){
            forces.push_back((fabs(cos(angles[i]*M_PI / 180)) + fabs((sin(angles[i]*M_PI / 180)))) / pow(distances[i], 2) * pParams->nRepulsiveScale);
	}
	else{
	    if(distances[i] >= 1500){
	    	forces.push_back(0);
	    }
	    else{
		forces.push_back((fabs(cos(angles[i]*M_PI / 180)) + fabs((sin(angles[i]*M_PI / 180)))) / pow(distances[i], 2) * pParams->nRepulsiveScale);
	    }
	}
    }
#if LOG_FORCES
    ArLog::log(ArLog::Normal, "--------------------- \n");    
#endif
    float nRepulsiveForceX, nRepulsiveForceY;    
    nRepulsiveForceX = 0;
    nRepulsiveForceY = 0;

    //update KinectContext and grab new position
    XnPoint3D position;
    int res = pKinectContext->GetPosition(position);
    
    if(res != -1){
    	for (int i = 0; i < pParams->nLaserDivisionAmount; i++) {
        	nRepulsiveForceX = nRepulsiveForceX + forces[i] * cos(angles[i]*M_PI / 180)*-1;
        	nRepulsiveForceY = nRepulsiveForceY + forces[i] * sin(angles[i]*M_PI / 180)*-1;
    	}
    }
    //get absolute robot position 
    double nRobotPositionX = pRobot->getX();
    double nRobotPositionY = pRobot->getY();
    
    double nTargetDistanceX;
    double nTargetDistanceY;
 
    //Updating of a position by Kinect
    if (res == -1)
    {
	//no tracked person
	nTargetDistanceX = 0;
	nTargetDistanceY = 0;
    }

    nTargetDistanceX = (double)position.Z;
    nTargetDistanceY = (double)position.X;
    
    //set attractive x and y forces
    if(nTargetDistanceX > 0){
		nRepulsiveForceX += pParams->nAttractionScale;
	}
	else{
		nRepulsiveForceX -= pParams->nAttractionScale;
	}
	
    if(nTargetDistanceY > 0){
		nRepulsiveForceY += abs(nTargetDistanceY/nTargetDistanceX) * pParams->nAttractionScale;
	}
	else{
		nRepulsiveForceY -= abs(nTargetDistanceY/nTargetDistanceX) * pParams->nAttractionScale;
	}

    //calculate angle to target
    double nRobotAngle = pRobot->getTh();     
    double nFinalAngle;

    pParams->nKinectAngle = atan(nRepulsiveForceY / nRepulsiveForceX);
    pParams->nKinectAngle = pParams->nKinectAngle * 180 / M_PI;
    if (nRepulsiveForceY < 0 && nRepulsiveForceX < 0){
        pParams->nKinectAngle = -180 + pParams->nKinectAngle;
    }
    if (nRepulsiveForceY > 0 && nRepulsiveForceX < 0){
        pParams->nKinectAngle = 180 + pParams->nKinectAngle;
    }

    nFinalAngle = nRobotAngle + pParams->nKinectAngle;
    
#if LOG_FORCES
    ArLog::log(ArLog::Normal, "f_x: %.4f ; f_y:  %.4f ; fin_ang: %.4f ; rob ang: %.4f\n", nRepulsiveForceX, nRepulsiveForceY, nFinalAngle, nRobotAngle);
#endif
    distances.clear();
    angles.clear();  
    forces.clear();
    double nTargetDistance = sqrt(pow(nTargetDistanceX, 2) + pow(nTargetDistanceY, 2));

    //printf("dist targ: %f\n",nTargetDistance);
    //set robot speed
    if((nTargetDistance > pParams->nStopDist) && (res != -1)){
        pDesired.setHeading(nFinalAngle);
	if(nTargetDistance < (pParams->nStopDist + 0.1*pParams->nStopDist )){
	    pDesired.setVel(200); //setting desired velocity of a robot
	}
	else{
	    pDesired.setVel(500);
	}
    }
    else{
	pDesired.setVel(0); //setting desired velocity of a robot
	if(res != -1){
	pDesired.setHeading(nFinalAngle);
	}
    }
    
    double resultant; 
    if(res!=-1){
        resultant = sqrt(pow(nTargetDistanceX, 2)+pow(nTargetDistanceY, 2));
        outfile.open("DistanceData.txt", std::ios_base::app);
        outfile << resultant << std::endl;
        outfile.close();

    }

    return &pDesired;
}
