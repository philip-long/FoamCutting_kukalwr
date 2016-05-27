
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
using namespace std;
/****************************************************************
 
This function checks if control law converges using joint position control. Firstly 
(a) Known object position
(b) Vision
(c) External force sensor

Q.
	(a) accurate?
	(b) smooth?
	(c) does the force cancelation improve accuracy ?

A.


*/

void ExternalForce(FastResearchInterface *FRI)
{


    
	/* 
	---------------------------------------------------------------------------------
	
		Declare  All Variables  

	---------------------------------------------------------------------------------		
	*/

	double timecounter=0;
	float ToolPose[12];	
	float MeasuredPose[12];
	float JointStiffnessValues[7];
  	float JointDampingValues[7]; 
	float JointValuesInRad[7];  // to store the joint value
	float dq[7];
	float MeasuredJointValuesInRad[7];
	float MeasuredJointTorqueValues[7];
	float DesiredTorqueValues[7];
	float Measuredforce[6];

	vpColVector fd(6);
	vpColVector	ResolvedForce(6);
	vpColVector Dxf(6);



	//initialize Jacobian matrix, 6*7, all 0
	vpMatrix J_model(6,7);
 	float **FriJaco;  //to store the measured Jacobian from KUKA
  	FriJaco = new float* [6];
  	for(int i=0;i<6;i++)
	FriJaco[i] = new float[7];
		/*
			Declare all Control law parameters 		
		*/


	// Parameters taken from file, can change to save data with different suffixs
    double lamda[7]; //regulation parameter in the control law
    double InterTime;// q(i+1)= q(i)+ qdot*InterTime


	//
	double TotalTime;
	double LamdaCart[6];


	vpColVector pSt(6);
	vpColVector pStdesire(6);
	vpColVector deltaS(6);	//feature s
	vpColVector V(6); // Kinematic Screw
	vpMatrix rdot(3,3); //Change in Rotation Matrix
	vpMatrix r(3,3); // Prenormalized Commanded Rotation matrix
	vpMatrix R(3,3); // Normalized Rotation matrix

	vpMatrix w_hat(3,3); // Skew symetric matrix of omega
	vpMatrix J(6,7); // Jacobian Matrix
	vpTranslationVector tTo; //Translation vector object to tool
	vpMatrix tTo_hat; tTo_hat.eye(3); // Skew symetric matrix of object to tool
	vpMatrix By,L;	L.eye(6);//interaction matrix L
	vpRotationMatrix tRo;//  Rotation matrix object  to tool
	vpRotationMatrix bRt,bRtdes; // Rotation matrix tool to base
	vpHomogeneousMatrix tMo,tMb,bMt,bMtdes,bMo;
	vpThetaUVector tthetauo;	 
	vpMatrix LA,LB,LC; // Blocks of the interaction matrix
	vpColVector v(3),w(3); // linear and angualr velocity respectively
	vpThetaUVector bthetaut;
	vpThetaUVector bthetautdes;
	// FRI 	Variables
	float cycletime;
	int ResultValue = 0;   

	GETParametersJ("./gain/ParametersJ", lamda, InterTime); // lamda is joint gain(unused)??
	



	  //recording the data
  	  //initialize write data
	ofstream OutputMeasuredX;
	ofstream OutputCommandedX;
	ofstream OutputMeasuredR;
	ofstream OutputCommandedR;
	ofstream OutputMeasuredForce;
	//ofstream OutputEstimatedForce;
	ofstream OutputDeltaS;
	//ofstream OutputDeltaF;
	//creat the file to write in
	OutputMeasuredX.open("./data/CI/MeasuredX");
	OutputDeltaS.open("./data/CI/deltaS");
	OutputCommandedX.open("./data/CI/CommandedX");
	OutputMeasuredR.open("./data/CI/MeasuredR");
	OutputCommandedR.open("./data/CI/CommandedR");
	OutputMeasuredForce.open("./data/CI/MeasuredForce");
	//OutputEstimatedForce.open("./data/CI/EstimatedForce");
	


	//OutputDeltaF.open("./data/CI/deltaF");      
	/* 
	---------------------------------------------------------------------------------
	---------------------------------------------------------------------------------		
	*/




//2. initialize the FRI


	

	if ((FRI->GetCurrentControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL) || (!FRI->IsMachineOK()))
	{	
		printf("Program is going to stop the robot.\n");
		FRI->StopRobot();			
		printf("Restarting the joint position control scheme.\n");
		ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
		
		if ((ResultValue != EOK) && (ResultValue != EALREADY))
			{
				printf("An error occurred during starting up the robot...\n");
			return;	
			}
	}


	cycletime = FRI->GetFRICycleTime();
 	
	GETParametersCI("./gain/ParametersCI", LamdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time


////////////////////////////////////4. now the loop where do the control
	

// Define Variables to test Vision Control Law without Camera
// This is the object location w.r.t base
   
bMo[0][0]=-0.082;
bMo[1][0]=0.997 ;
bMo[2][0]=0.004 ;
bMo[3][0]=0.000 ;
bMo[0][1]=0.986;
bMo[1][1]= 0.081; 
bMo[2][1]=0.145 ;
bMo[3][1]=0.000 ;
bMo[0][2]=0.145 ;
bMo[1][2]=0.016 ;
bMo[2][2]=-0.989;
bMo[3][2]=0.000;
bMo[0][3]=0.016;
bMo[1][3]=0.599;
bMo[2][3]=0.0970;
bMo[3][3]=1.000;

	while ((FRI->IsMachineOK()) && timecounter<TotalTime)
	{
		timecounter+=0.02;

		
		// Measure and convert data from FRI
		FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
		FRI->GetMeasuredJointTorques(MeasuredJointTorqueValues);
		FRI->GetMeasuredCartForcesAndTorques(Measuredforce);

		FRI->WaitForKRCTick();

		FRI->GetCurrentJacobianMatrix(FriJaco);
		FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the tool frame
 
		FRI->GetMeasuredCartPose(MeasuredPose);
		FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt); 

		

		// Define S and deltaS
		bMt.extract(bRt);//Rotation matrix R from HomogeneousMatrix
		bthetaut.buildFrom(bRt); // u theta Representation of Orientation
		printfM(bMt,"\n Current Tool to Base: \n");

		tMo=bMt.inverse()*bMo;// Defining s for vision test
		printfM(tMo,"\n Object in Tool Frame: \n");
		tMo.extract(tRo);//Rotation matrix R from HomogeneousMatrix
		tMo.extract(tTo);//extract the deplacement vector
		tthetauo.buildFrom(tRo);//theta u representation
		Two3dimensionVector2OnedemensionVector(tTo,tthetauo,pSt);
		
		deltaS = pSt-pStdesire;
		printfVector(deltaS,"\n Error deltaS=",6);
		double NormS;
		NormS=VectorSqrt(deltaS,6);
		printf("\n NormS =%8.3f \n",NormS);
		if((VectorSqrt(deltaS,3)<0.015)&(VectorSqrt(deltaS,6)<0.1))
			{
			printf("Location reached\n");
			}



		/* 
		---------------------------------------------------------------------------------
		
				Compute the correctional velocity

		---------------------------------------------------------------------------------		
		*/

		SkewSym(tTo,tTo_hat); // computes the skew symetric matrix

		//get the current interaction matrix L
		ComputeLw(tthetauo,By); // Calculates Matrix to convert omega to uthetadot
		
		/*
				 Build the interaction matrix by 3x3 block 
					L= [A	B]
						0	C]
		*/


		LA.eye(3);
		LB=-tTo_hat;
		LC.eye(3);


		for (int i=0;i<3;i++)
			{			
			
			for(int j=0;j<3;j++)
				{

				L[i][j]=LA[i][j]; // Upper  left  
				L[i+3][j]=0;// Lower left
				L[i][j+3]=LB[i][j];// Upper right
				L[i+3][j+3]=LC[i][j];// Lower right

				}
 
			}


	printf("\nL=[");
		for (int i=0;i<6;i++)
		{
		printf("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f \n",L[i][0],L[i][1],L[i][2],L[i][3],L[i][4],L[i][5]);
		}		
	printf("]");

		
		
		V = (-L).pseudoInverse()*deltaS;
		// Applying the gain to the Cartesian velocity, which should be in the base frame
		for(int i=0;i<6;i++)
		{
		V[i]=-LamdaCart[i]*V[i]; // Applying gain to Cartesian Directions instead of Joint
		
		}
		printf("\n Velocity in Tool Frame Space:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
		

		/*
		---------------------------------------------------------------------------------
		
			 Applying Force Deviation

		---------------------------------------------------------------------------------		
		*/
		for(int i=0;i<6;i++)
		 {					
			Dxf[i]=lamda[i]*(fd[i] + (InterTime *(fd[i]-ResolvedForce[i])));
			V[i]=V[i]-Dxf[i];
		} 
	
		printf("\n Velocity after Force deviation Frame Space:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
	
		V=J.pseudoInverse()*V;

		/*
		---------------------------------------------------------------------------------
		
				Integrate qdot to calculate the q

		---------------------------------------------------------------------------------		
		*/

		 for(int i=0;i<7;i++)
			 {
			dq[i]=V[i];
			JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (cycletime*dq[i]);
			 } 




		/* 
		---------------------------------------------------------------------------------		
		---------------------------------------------------------------------------------		
		*/





		/* 
		---------------------------------------------------------------------------------
		
				Send Data to Robot for Cartesian Control

		---------------------------------------------------------------------------------		
		*/



		printf("\n Cycle time=,%8.3f",cycletime);
		printf("\n Lambda=,%8.3f, %8.3f, %8.3f ,%8.3f, %8.3f, %8.3f, %8.3f\n",LamdaCart[0],LamdaCart[1],LamdaCart[2],LamdaCart[3],LamdaCart[4],LamdaCart[5],LamdaCart[6]);

	

		FRI->SetCommandedJointTorques(DesiredTorqueValues);
		FRI->SetCommandedJointDamping(JointDampingValues);
 		FRI->SetCommandedJointStiffness(JointStiffnessValues);
		FRI->SetCommandedJointPositions(JointValuesInRad);
 
		printf("timecounter %f \n",timecounter);	
		printf("\n ---------------------------------------------- \n");	




		/* 
		---------------------------------------------------------------------------------
		
				Saving Data for postprocessing

		---------------------------------------------------------------------------------		
		*/
		ZeroForceSensor(bMt,Measuredforce,ResolvedForce);
		FRICartPose2vpHomogeneousMatrix(ToolPose,bMtdes); 
		bMtdes.extract(bRtdes);
		bthetautdes.buildFrom(bRtdes);

	    for(int i=0;i<3;i++)
	    {
	    OutputMeasuredX << MeasuredPose[i*4+3] << " ";  //save cart pose
	    }
	    OutputMeasuredX << endl;

	    for(int i=0;i<3;i++)
	    {
	    OutputCommandedX << ToolPose[i*4+3] << " ";  //save cart pose
	    }
	    OutputCommandedX << endl;

			    for(int i=0;i<3;i++)
	    {
	    OutputMeasuredR << bthetaut[i] << " ";  //save cart pose
	    }
	    OutputMeasuredR << endl;

	    for(int i=0;i<3;i++)
	    {
	    OutputCommandedR << bthetautdes[i] << " ";  //save cart pose
	    }
	    OutputCommandedR << endl;


		for(int i=0;i<6;i++)
		{
		OutputDeltaS<< deltaS[i] << " ";  //save cart pose
		}
		OutputDeltaS << endl;

		for(int i=0;i<6;i++)
		{
		OutputMeasuredForce<< ResolvedForce[i] << " ";  //save cart pose
		}
		OutputMeasuredForce << endl;
		

/////////////////////////////////////////////////////////////////////////////data saving finished



	}

	printf("Time Over\n");
	

	if (!FRI->IsMachineOK())
	{
		printf("RunTrajectorySimple(): ERROR, machine is not ready.");
		

		return;
	}

	printf("Stopping the robot.\n");
	FRI->StopRobot();	

	OutputMeasuredX.close();
	OutputCommandedX.close();
	OutputMeasuredForce.close();
	OutputDeltaS.close();

	return;
}





/*

	Code to save data with name defined to external file parameters, to facilate testing
	Saving data with a different suffix correspong to user input
	


	sprintf(W_or_utheta,"%f",lamda[0]); // 	Omega ==1
	sprintf(Jinv_or_JT,"%f",lamda[1]); //   Jacobian Tranpose ==1
	sprintf(RunNum,"%f",lamda[2]); // Run Number		
	char destS[50];
	char* str="./data/CI/deltaS_";
	strcpy(destS,str);
	strcat(destS,W_or_utheta);	
	strcat(destS,"_");
	strcat(destS,Jinv_or_JT);
	strcat(destS,"_");
	strcat(destS,RunNum);	


	char destX[50];
	char* strX="./data/CI/MeasuredX_";
	strcpy(destX,strX);
	strcat(destX,W_or_utheta);	
	strcat(destX,"_");
	strcat(destX,Jinv_or_JT);
	strcat(destX,"_");
	strcat(destX,RunNum);	

	char destR[50];
	char* strR="./data/CI/MeasuredR_";
	strcpy(destR,strR);
	strcat(destR,W_or_utheta);	
	strcat(destR,"_");
	strcat(destR,Jinv_or_JT);
	strcat(destR,"_");
	strcat(destR,RunNum);	
	char W_or_utheta[10];
	char Jinv_or_JT[10];
	char RunNum[10];
	OutputDeltaS.open(destS);
	OutputMeasuredX.open(destX);
	OutputMeasuredR.open(destR);

*/	
