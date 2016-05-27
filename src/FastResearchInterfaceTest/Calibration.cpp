
#include <FunctionLibrary.h>
#include <ViSPConfiguration.h>

//fri include
#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>

#include <visp/vpVideoWriter.h>
// ViSP includes
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpPose.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpTrackingException.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImageTools.h>
#include <visp/vpMatrix.h>


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <fstream>

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

#define NUMBER_OF_POSE			12
#define NUMBER_OF_JOINTS		7
using namespace std;
/*  ****************************************************************

	Aim of this function is twofold, 
	1. Firstly it aims to gather all the data required to find
    the trasnformation matrix from the camera to the gripper using a calibration Object
	The function saves cMo the location of the object with respect to the camera and also wMe
    the frame of the robot end effector with respect to the world. A least squares optimization problem is formulated 
   offline --> [Tsai and Lenz 1989] and the eMc can be calculated

	2. We want to store the force data and robot location in order to calibrate the force sensor for this particular load.
	It is assumed that the transformation from the end effector to the tool frame is known as is the transformation from the
   force sensor to the robot end effector
	Once we have all the data, the parameters to cancel the effects of the tool at any location can be calculated online. 
	Note this will only compensate for STATIC loads!

**************************************************************** */


void Calibration(FastResearchInterface *FRI)
{	

int CalibrateChoice;


//CalibrateChoice=1;
CalibrateChoice=0;
/*-------------------------------------------------------------------------------------------------------

Declare all variable

-------------------------------------------------------------------------------------------------------*/


	
	float Measuredforce[6];

	float Measuredforce2[6];
	float MeasuredPose[12];
	vpColVector	ResolvedForce(6);
	vpColVector ResolvedForce2(6);
	vpHomogeneousMatrix cMo,bMt,eMt,bMe;
	float timecounter=0.0;
    float TotalTime=500; // Number of iterations per position
// Saving the ouput variables -Force, Transformation for object to Camera and the robot transformation

   GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame
  


 ofstream OutputMeasuredcMo; OutputMeasuredcMo.open("./data/Calibration/cMo",std::ios::out | std::ios::trunc);
  ofstream OutputMeasuredbMe; OutputMeasuredbMe.open("./data/Calibration/bMe",std::ios::out | std::ios::trunc );
  ofstream OutputMeasuredbMt; OutputMeasuredbMt.open("./data/Calibration/bMt",std::ios::out | std::ios::trunc );
  ofstream OutputMeasuredForce; OutputMeasuredForce.open("./data/Calibration/Force",std::ios::out | std::ios::trunc );

   ofstream OutputMeasuredcMobk;   OutputMeasuredcMobk.open("./data/Calibration/cMo.bak",std::ios::out | std::ios::app);
   ofstream OutputMeasuredbMebk;   OutputMeasuredbMebk.open("./data/Calibration/bMe.bak",std::ios::out | std::ios::app );
   ofstream OutputMeasuredForcebk; OutputMeasuredForcebk.open("./data/Calibration/Force.bak",std::ios::out | std::ios::app );

//************************************* Moving Joints Automatically VARIABLES**********************************************************************

// --------------------------------------------------------------------------------------------------------

// ---------------------------------------------- Trajecory Test ------------------------------------------
float MeasuredJointValuesInRad[7];
float JointValuesInRad[7];
FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

vpColVector qinit(7);
vpColVector qt(7);
for (int j=0;j<7;j++)
{
	qinit[j]=MeasuredJointValuesInRad[j];
	qt[j]=MeasuredJointValuesInRad[j];
}
printf(" qt intial =[ %8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f] \n",qt[0],qt[1],qt[2],qt[3],qt[4],qt[5],qt[6]);	

		
//************************************* OUTPUT VARIABLES**********************************************************************


/*************************************************************************************************************

Start the data gathering process


*************************************************************************************************************/


  while((FRI->IsMachineOK()) && timecounter<TotalTime)
  {
  
	
   	// First do all the operations for all Control Stages
	timecounter+=0.2;
	FRI->WaitForKRCTick();
    printf("Counter=%f\n",timecounter);

	
	TrajecSects(qinit,qt,timecounter);
		
	 printf(" qt =[ %8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f] \n",qt[0],qt[1],qt[2],qt[3],qt[4],qt[5],qt[6]);	
		

	FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);
    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);
	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

	// Putting in a simple motion here specifically for force measurement
	 
	bMe=bMt*eMt.inverse();
	for (int j=0;j<7;j++)
	{
		JointValuesInRad[j]=qt[j];
	}

	FRI->SetCommandedJointPositions(JointValuesInRad);


		for(int i=0;i<4;i++)
		{
			OutputMeasuredbMt<< bMt[i][0] << " ,"; 
			OutputMeasuredbMt<< bMt[i][1] << " ,"; 
			OutputMeasuredbMt<< bMt[i][2] << ", "; 
			OutputMeasuredbMt<< bMt[i][3] << " "; 
			OutputMeasuredbMt<<endl;
		}

		for(int i=0;i<4;i++)
		{
			OutputMeasuredbMe<< bMe[i][0] << " ,"; 
			OutputMeasuredbMe<< bMe[i][1] << " ,"; 
			OutputMeasuredbMe<< bMe[i][2] << ", "; 
			OutputMeasuredbMe<< bMe[i][3] << " "; 
			OutputMeasuredbMe<<endl;
		}

		for(int i=0;i<6;i++)
		{
		OutputMeasuredForce<< Measuredforce[i] << " ";  //save Force Sensor
		
		}
		OutputMeasuredForce <<endl;
		for(int i=0;i<4;i++)
		{
			OutputMeasuredcMobk<< cMo[i][0] << ", "; 
			OutputMeasuredcMobk<< cMo[i][1] << " ,"; 
			OutputMeasuredcMobk<< cMo[i][2] << ", "; 
			OutputMeasuredcMobk<< cMo[i][3] << " "; 
			OutputMeasuredcMobk<< endl;
		}


		for(int i=0;i<4;i++)
		{
			OutputMeasuredbMebk<< bMe[i][0] << " ,"; 
			OutputMeasuredbMebk<< bMe[i][1] << " ,"; 
			OutputMeasuredbMebk<< bMe[i][2] << ", "; 
			OutputMeasuredbMebk<< bMe[i][3] << " "; 
			OutputMeasuredbMebk<<endl;
		}

		for(int i=0;i<6;i++)
		{
		OutputMeasuredForcebk<< Measuredforce[i] << " ";  //save Force Sensor
		
		}
		OutputMeasuredForcebk <<endl;
  }
  

	FRI->StopRobot();	
	OutputMeasuredcMo.close();
	OutputMeasuredbMe.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputMeasuredcMobk.close();
	OutputMeasuredbMebk.close();
	OutputMeasuredForcebk.close();

}





















