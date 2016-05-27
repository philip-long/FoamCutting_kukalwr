//  ---------------------- Doxygen info ----------------------
//! \file UserDefinedTrajectory.cpp
//!
//! \brief
//! Read the external file and pass the trajecory in joint space to 
//! Kuka robot
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <ScriptLibrary.h>
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

// ****************************************************************
// UserDefinedTrajectory()
//
void DisplayData(FastResearchInterface *FRI)
{

    printf("Here 1");
	ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./DisplayData/bMt",std::ios::out | std::ios::app  );
    ofstream OutputMeasuredbMe;      OutputMeasuredbMe.open("./DisplayData/bMe",std::ios::out | std::ios::app  );
    printf("Here 2");
	unsigned int i,j,k;
    vpHomogeneousMatrix bMt,bMe,eMt;
	float	JointValuesInRad[NUMBER_OF_JOINTS],FloatArray[12];

	float 	JacobianArray[42];
	float 	JacobianMatrix[6][7];
    float   Jtranpose[7][6];
    
	float 	MsrJntTorques[NUMBER_OF_JOINTS],EstExtJntTorques[NUMBER_OF_JOINTS],EstExtCartForces[NUMBER_OF_JOINTS],ANS[10];

    GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame

    printf("\n eMt=");printfM(eMt);
	//1. Joint Positions
	FRI->GetMeasuredJointPositions(JointValuesInRad);
	
	for (i = 0; i < LBR_MNJ; i++)
		{
		printf("Joint position %d: %8.3f degrees\n", i, JointValuesInRad[i] * 180.0 / PI);
		}

	//2. Cartesian Pose
	printf("Cartesian Pose\n");
	FRI->GetMeasuredCartPose(FloatArray);
    FRICartPose2vpHomogeneousMatrix(FloatArray,bMt);
    bMe=bMt*eMt.inverse();
	printf("%8.3f  %8.3f   %8.3f  %8.3f  \n",FloatArray[0],FloatArray[1],FloatArray[2],FloatArray[3]);
	printf("%8.3f  %8.3f   %8.3f  %8.3f  \n",FloatArray[4],FloatArray[5],FloatArray[6],FloatArray[7]);
	printf("%8.3f  %8.3f   %8.3f  %8.3f  \n",FloatArray[8],FloatArray[9],FloatArray[10],FloatArray[11]);
	printf("%8.3f  %8.3f   %8.3f  %8.3f  \n",0.0,0.0,0.0,1.0);

	

	// 5. 	TORQUE_AXIS_ACT - Axis Specific Torque --> tau_{FRI}
	//msr.data.msrJntTrq
	FRI->GetMeasuredJointTorques(MsrJntTorques);
	printf("Measured Joint Torques ($TORQUE_AXIS_ACT)=\n [");		
	for (i = 0; i < LBR_MNJ; i++)
		{
		printf("%f",MsrJntTorques[i]);
		}
	printf("]\n");	
		
	//6. Estimated External Axis Specific Torque	--> tau_{cmd}=tau_{FRI}+ K delta(q)+ D(d) + f(dynamics)
	//msr.dat.estExtJntTrq
	//Gettable but not settable	
	FRI->GetEstimatedExternalJointTorques(EstExtJntTorques);
	printf("Estimated External Joint Torques($TORQUE_AXIS_EST )=\n[") ;
	for (i = 0; i < LBR_MNJ; i++)
		{
		printf("%8.3f:", EstExtJntTorques[i]);
		}
	printf("]\n");		
	
	
	// 7. Estimated External Cartesian Forces and Torques -->
	//mst.data.estExtTcpFt
	FRI->GetEstimatedExternalCartForcesAndTorques(EstExtCartForces);
	printf(" Get Estimated External Cartesian Forces and Torques($TORQUE_TCP_EST)=\n[");
	printf("[");
	for (i = 0; i < 6; i++)
		{
		printf("%8.3f,", EstExtCartForces[i]);
		}
	printf("]\n");

	// 8. Calculation to check
	//Multipy J' x F
for(i=0;i<7;i++)
	{
	for(j=0;j<7;j++)
		{

		ANS[i]=0;
			for(k=0;k<6;k++)
			{	
			ANS[i]=ANS[i]+(Jtranpose[i][k]*EstExtCartForces[k]);			
			}
		}
	}

		printf("\n J'xF =[: %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f %8.3f]",ANS[0],ANS[1],ANS[2],ANS[3],ANS[4],ANS[5],ANS[6]);
			printf("]\n");

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
			return;
}


