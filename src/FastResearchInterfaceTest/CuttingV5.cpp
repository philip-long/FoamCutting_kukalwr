
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
/*  ****************************************************************

	Aim of this function is twofold, 
    
	1. Trying the slice and press method of cutting something
        % Input a straight line to cut at a certain depth (passage)
        % Try to cut if force becomes to start to slice 
        % If force drops below a threshold slice finished go to next passage


**************************************************************** */


void CuttingV5(FastResearchInterface *FRI)
{	

/*-------------------------------------------------------------------------------------------------------

Declare all variable

-------------------------------------------------------------------------------------------------------*/

float dq[7];
float cycletime;

float **FriJaco;  //to store the measured Jacobian from KUKA
FriJaco = new float* [6];
for(int i=0;i<6;i++)
FriJaco[i] = new float[7];
float Measuredforce[6];
float Measuredforce2[6];
float MeasuredPose[12];
float timecounter; // actual time spent
float timetraj,timetrajfinal;
timetraj=0.0;
timetrajfinal=50.0;

double TotalTime;
float MeasuredJointValuesInRad[7];
float JointValuesInRad[7];
cycletime = FRI->GetFRICycleTime();

/*-------------------------------------------------------------------------------------------------------

ViSP VARIABLES

-------------------------------------------------------------------------------------------------------*/

vpQuaternionVector rQ;
vpColVector bVt(3),Vbase(6),tV(6),bOmegat(3),qdot(6),qinit(7),qt(7);
vpMatrix L(6,6);
vpMatrix J(6,7); // Jacobian Matrix
vpThetaUVector Sthetau;
vpRotationMatrix bRt,R_S,cutRt,tRcut,tRb;
vpHomogeneousMatrix bMt_init,bMt,tMb,bMt_d,S, bMtdesired,bMtTraj,bMtlast,diffM,bMc;
vpTranslationVector T_S;




// Subsection: Cutting variables supplied by an as yet imaginary cutting system
float CuttingDepth;    float CutData[4];
vpHomogeneousMatrix bMts1,bMtpassage,bMclast;

float Zmax=0.120; // this should be the maximum height of the block at any given x-y

int Move=0;






//************************************* INPUT VARIABLES**********************************************************************
    vpHomogeneousMatrix cMt,tMc,eMf,eMt,cutMt,tMcut;
    double Kf[7];  // Force gain
    double Kfi[6];
    float temp_x[11]; // A temporary variable 
    double LambdaCart[6];
    double Var1;
    double Var2;
    double DesiredForce[6]; // Superimposed force
    double Kdeltaf; // Gain multiplied by force error
     GETHomogeneousMatrix(tMc,"./ConstantMatrices/tMc");  //Camera to Tool frame
    GETHomogeneousMatrix(cutMt,"./ConstantMatrices/cutMt");  //Camera to Tool frame
    GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
    GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame
    printf("cutMt=\n");printfM(cutMt);
    GETParametersCI("./CuttingV5/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
    GETParametersCI("./CuttingV5/Kf", Kf, Var1); //Using these parameters as misc for debug
    GETParametersCI("./CuttingV5/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
    tMcut=cutMt.inverse();
    cutMt.extract(cutRt);

//************************************* FORCE VARIABLES**********************************************************************
FILE * pFile;pFile=fopen("./CuttingV5/IdentifiedForceParameter","rb");
float MovingAverageForce[6];
float Dxf[6]; // The deviation cause by the force

float OmegaForce[3];
float EulerIntegrator[6];
vpMatrix ForceBuffer(5,6);
vpColVector x(11); // Idenitifed mass parameters of force sensor
vpColVector	ResolvedForce(6);
vpColVector ResolvedForce2(6);

for (int i=0;i<6;i++)
{
DesiredForce[i]=0.0;
}
fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7],&temp_x[8],&temp_x[9],&temp_x[10]);
//x[0]=8.3335;x[1]=1.1624;x[2]=-2.4131;x[3]=0.8784;x[4]=-5.974;x[5]=0.025479;x[6]=0.50777;x[7]=-0.020928;x[8]=0.012594;x[9]=0.31904;	

for (int i=0;i<11;i++)
{
x[i]=temp_x[i];
}


printf("\n X=%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[9]);



//************************************* OUTPUT VARIABLES**********************************************************************

 

   

    ofstream OutputS;     OutputS.open("./CuttingV5/Results/S",std::ios::out | std::ios::app );
    ofstream OutputWork;     OutputWork.open("./CuttingV5/Results/CutData",std::ios::out | std::ios::app );
    ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingV5/Results/bMt",std::ios::out | std::ios::app );
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingV5/Results/MeasuredForce");
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingV5/Results/ResolvedForce");
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingV5/Results/MovingAverageForce");

//*************************************END OF DECLARATIONS**********************************************************************



// Get initial positions
FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt_init);
 bMts1=bMt_init;
 bMts1[3][3]=bMt_init[3][2]+0.05;

float CuttingAngle=PI/4;
printf("\n CuttingAngle=%f");


/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/


printf("\nStart\n");
printf("Is machine ok=%d\n",FRI->IsMachineOK());
printf("TotalTime  =%f\n",TotalTime);


  for (int i = 0; i < 200; i += 1)
  {
      FRI->WaitForKRCTick();
      printf("i=%d \n",i);
  }
  while((FRI->IsMachineOK()) && timecounter<TotalTime)
  {
    printf("Counter=%f\n",timecounter);
  	timecounter+=cycletime;
    timetraj+=cycletime;
	FRI->WaitForKRCTick();

    /*************************************************************************************************************

    Get the data from the FRI

    *************************************************************************************************************/

    FRI->GetCurrentJacobianMatrix(FriJaco);
    FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame
    bMtlast=bMt; // Save the last pose
	FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);
    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);
	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

   
   //XTrajecSim(bMtpassage,bMtdesired,bMtTraj,timetraj,timetrajfinal,bVt,bOmegat);


    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

					                FORCE CONTORL 

    1. Find the net contact forces at the TCP
    2. Insert the most recent into a buffer (stack)
    3. Low pass filter the force data to limit noisy measurements

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
	

    bMt.extract(bRt);
    tMb=bMt.inverse();tMb.extract(tRb);

    ResolveForceatTCP( bMt, eMf, eMt,Measuredforce,ResolvedForce,x);
    // Have to change the frame now to the cutting frame quite disasterously

    RotateScrew(ResolvedForce,ResolvedForce2,bRt);
    fifoBufferInsert(ForceBuffer,ResolvedForce2);
    LowPassFilter(ForceBuffer,MovingAverageForce);

    printf("\n Resolved Force=");printfM(ResolvedForce);
    printf("\n Resolved Force in world frame=");printfM(ResolvedForce2);
    

  /*************************************************************************************************************

    Check the status of the controller

    *************************************************************************************************************/

    int Cutting=0;
    if(bMt[2][3]>Zmax) // if outside body or in reset mode  RESET 
    {

        printf(" Exceeded maximum height");
        printf("\n\n\nRESETING \n \n \n");
        printf("\n\n\n\nRESETING \n \n \n  \n ");
  
        MovePoint(FRI,bMts1,5.0); // Move to safe point
        MovePoint(FRI,bMt_init,5.0); // Move to begin Passage
    }
    else
    {
        printf("\n Cutting \n");

    /*------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                    Velocity CONTORL , in world frame always ok
    

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */

        T_S[0]=0.1;
        T_S[1]=0.0; 
        T_S[2]=0.0;
        Sthetau[0]=0.0;
        Sthetau[1]=0.0;
        Sthetau[2]=0.0;   

    /*************************************************************************************************************

    Create force alterations also in the world frame
        Resistive force induces a slicing motion
        Resistive force also 
    *************************************************************************************************************/
    

        printf("Velocity pre deviation =\n");printfM(T_S);
        //if(MovingAverageForce[1]>5) // If the force is greater than threshold decrease desired  depth
        //{
       T_S[2]=(-0.025*MovingAverageForce[0]); // slicing motion 
      // Question is what to do with y velocity I think I should reduce it, we can play a little bit with it  
       T_S[0]=T_S[0]+(0.025*MovingAverageForce[0]);
        //}
        printf("Velocity post deviation =\n");printfM(T_S);
    
    /*************************************************************************************************************

    Calculate the cutting parameters

    *************************************************************************************************************/
    // Work done by cutting forces, Calculate the fracture toughness


    // Get tool position        
    bMclast=bMtlast*tMcut;    bMc=bMt*tMcut;
    diffM=bMc*bMclast.inverse();
    printf("Difference between current and last=\n");printfM(diffM);

    CutData[0]= diffM[1][3]*MovingAverageForce[1]; // Work by cut 
    CutData[1]= diffM[2][3]*MovingAverageForce[2]; // Work by slice
    CutData[2]=Zmax-bMc[2][3]; // Contact width
    CutData[3]=diffM[1][3];// Crack propogration
    CutData[4]=(CutData[0]+CutData[1])/(CutData[2]*diffM[1][3]);
    printf("Supposed Fracture toughness=%f",CutData[4]);

    /*************************************************************************************************************

    Define Error between bMt_d and bMt in base frame and Compute controller. V=Vbase 

    *************************************************************************************************************/
  

    for(int i=0;i<3;i++)
		{			
			for(int j=0;j<3;j++)
			{
			tRcut[j][i]=cutRt[i][j];
			}
		}   
    

    for(int i=0;i<3;i++)
    {	
    Vbase[i]=LambdaCart[i]*T_S[i];
    Vbase[i+3]=LambdaCart[i+3]*Sthetau[i];
    } 

    RotateScrew(Vbase,tV,tRb);

    printf("\n V in tool frame");printfM(tV);
	qdot=J.pseudoInverse()*tV;

    /*************************************************************************************************************

    Send to FRI

    *************************************************************************************************************/

    for(int i=0;i<7;i++)
    {
        dq[i]=qdot[i];
        JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (dq[i]*cycletime);
    //    JointValuesInRad[i]=qt[i];
    } 

    FRI->SetCommandedJointPositions(JointValuesInRad);
	

    }// End the if block





    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    

	// Writing Data to file
  
    for(int i=0;i<3;i++)
    {
        OutputS<< T_S[i] << " ";  //save cart pose
    }
    
    for(int i=0;i<3;i++)
    {
        OutputS<< Sthetau[i] << " ";  //save cart pose
    }
    OutputS << endl;
    
    for(int i=0;i<5;i++)
    {
        OutputWork<< CutData[i] << " ";  //save cart pose
    }
    OutputWork << endl;

    

	for(int i=0;i<4;i++)
	{
		OutputMeasuredbMt<< bMt[i][0] << " ,"; 
		OutputMeasuredbMt<< bMt[i][1] << " ,"; 
		OutputMeasuredbMt<< bMt[i][2] << ", "; 
		OutputMeasuredbMt<< bMt[i][3] << " "; 
		OutputMeasuredbMt<<endl;
	}
    
    // Measured Force raw from force sensor



    for(int i=0;i<6;i++)
    {
    OutputMeasuredForce<< Measuredforce[i] << " ";  //save Force Sensor
    }
    OutputMeasuredForce << endl;

    for(int i=0;i<6;i++)
    {
    OutputResolvedForce<< ResolvedForce2[i] << " ";  //save Force Sensor
    }
    OutputResolvedForce << endl;
  

    for(int i=0;i<6;i++)
    {
        OutputMovingAverageForce<< MovingAverageForce[i] << " ";  //save Force Sensor
    }
    OutputMovingAverageForce << endl;
  

    printf("\n time Counter=%f \n",timecounter);
    printf("\n-----------------------------------------------------------------------\n");
    // End of loop

  }
  




	FRI->StopRobot();	
	OutputS.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputResolvedForce.close();
	OutputMovingAverageForce.close();
}





















