
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


Aim of this function is to gather comparison data for new cutting control law


**************************************************************** */


void CuttingComparison(FastResearchInterface *FRI)
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
vpRotationMatrix bRt,bRtd,bRtdT,R_S,cutRt,tRcut,tRb;
vpHomogeneousMatrix bMtinit,bMtFinal,bMt,tMb,bMt_d,S,bMtsafe,bMtsafe2, bMtdesired,bMtTraj,bMtlast,diffM,bMc;
vpTranslationVector T_S,bPtd,bPt;



float tcut=0.0;
int Cutting=0;

// Subsection: Cutting variables supplied by an as yet imaginary cutting system
float CuttingDepth;    float CutData[4];
vpHomogeneousMatrix bMts1,bMtpassage,bMclast;

float Zmax=0.120; // this should be the maximum height of the block at any given x-y

int Move=0;






//************************************* INPUT VARIABLES**********************************************************************
    vpHomogeneousMatrix cMt,tMc,eMf,eMt,cutMt,tMcut,CurveStart;
    double Kf[7];  // Force gain
    double Kfi[6];
    float temp_x[11]; // A temporary variable 
    double LambdaCart[6];
    double Var1;
    double Var2;
    double DesiredForce[6]; // Superimposed force
    double Kdeltaf; // Gain multiplied by force error

    GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
    GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame

    GETParametersCI("./CuttingComparison/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
    GETParametersCI("./CuttingComparison/Kf", Kf, Var1); //Using these parameters as misc for debug
    GETParametersCI("./CuttingComparison/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
    tMcut=cutMt.inverse();
    cutMt.extract(cutRt);

//************************************* FORCE VARIABLES**********************************************************************
FILE * pFile;pFile=fopen("./CuttingComparison/IdentifiedForceParameter","rb");
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

 

   

    ofstream OutputS;     OutputS.open("./CuttingComparison/Results/S",std::ios::out | std::ios::trunc );
    ofstream OutputSliceParams;     OutputSliceParams.open("./CuttingComparison/Results/SliceGain",std::ios::out | std::ios::trunc );
    ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingComparison/Results/bMt",std::ios::out | std::ios::trunc );
    ofstream OutputMeasuredbMtd;      OutputMeasuredbMtd.open("./CuttingComparison/Results/bMtd",std::ios::out | std::ios::trunc );
    ofstream OutputCmdtV;      OutputCmdtV.open("./CuttingComparison/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingComparison/Results/MeasuredForce",std::ios::out | std::ios::trunc );
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingComparison/Results/ResolvedForce",std::ios::out | std::ios::trunc );
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingComparison/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

//*************************************END OF DECLARATIONS**********************************************************************



/*************************************************************************************************************

Initialising the robot

*************************************************************************************************************/
FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);



printf("\n bMtinit=\n");printfM(bMtinit);

// Eventually both of these will be supplied by camera

float MaterialHeight=.117;  
printf("TotalTime  =%f\n",TotalTime);
printf("cycletime=%f\n",cycletime);
//float p[2];
//p[0]=-0.53794;p[1]=-0.17549;p[2]=0.06475;


float CuttingAngle=PI/6;
printf("\n CuttingAngle=%f");
vpHomogeneousMatrix tMtc,bMtd,Mtest,R1,R2,R3;
    for (int i = 0; i < 3; i += 1)
    {
        Mtest[i][3]=bMtinit[i][3];    
    }


//TransMat(p,Mtest); printf("Mtest=\n");


TransMat(1,-PI,R1);TransMat(2,0,R2);TransMat(3,-PI,R3);
Mtest=Mtest*R1*R2*R3;
printfM(Mtest);

TransMat(1,CuttingAngle,tMtc);
CurveStart=Mtest; CurveStart=CurveStart*tMtc;
bMtsafe=CurveStart;
bMtsafe[2][3]=.15;

bMtFinal=bMtinit;
bMtFinal[1][3]=0.1; // Final in Y position

bMtFinal.extract(bPtd);   
bMtFinal.extract(bRtd);
/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/


printf("\nStart\n");
printf("Is machine ok=%d\n",FRI->IsMachineOK());
printf("TotalTime  =%f\n",TotalTime);
float SliceGain=0.0;
float tfcut=50.0;

printf("bMtfinal=");
printfM(bMtFinal);
  for (int i = 0; i < 20; i += 1)
  {
      FRI->WaitForKRCTick();
      printf("i=%d \n",i);
  }
    OutputSliceParams<< SliceGain << " ";  //Cart Angular Velocity base
    OutputSliceParams<< tfcut << " ";  //Cart Angular Velocity base
    OutputSliceParams<< CuttingAngle << " ";  //Cart Angular Velocity base
    OutputSliceParams << endl;
 	OutputSliceParams.close();
    MovePointExp(FRI,CurveStart,2,0.5,0.002,0.01); // Move to begin Passage

     while((FRI->IsMachineOK()) && timecounter<300.0)
    {


    FRI->WaitForKRCTick();


  	timecounter+=cycletime;


    /*************************************************************************************************************

    Get the data from the FRI

    *************************************************************************************************************/

    FRI->GetCurrentJacobianMatrix(FriJaco);FRIJaco2vpMatrix(FriJaco,J); 

	FRI->GetMeasuredCartPose(MeasuredPose);    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);

    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);

	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

    bMt.extract(bRt);    tMb=bMt.inverse();tMb.extract(tRb);

    tMb=bMt.inverse();tMb.extract(tRb);
    bMt.extract(bRt);bMt.extract(bPt);

    /*************************************************************************************************************

    Check the status of the controller

    *************************************************************************************************************/
   
    if(bMt[2][3]>MaterialHeight) // if outside body or in reset mode  
    {
    Cutting=0;
    tcut=0;
    }
    else
    {
    Cutting=1;
    }


     switch (Cutting){
        case 0: // Not cutting
            printf(" Exceeded maximum height"); 
            printf("\n Z is %f,while X is %f",bMt[2][3],bMt[0][3]);
            printf("\n\n\nRESETING \n \n \n");
            printf("\n Move to Safe Point bMtsafe2");
            bMtsafe2=bMt;
            bMtsafe2[2][3]=bMt[2][3]+0.035;
            MovePointExp(FRI,bMtsafe2,7.0,1,0.01,0.5); // Move to safe point
            printf("\n Move to Safe Point bMtsafe");
            MovePointExp(FRI,bMtsafe,2.0,1,0.01,0.4); // Move to safe point
            printf("\n Move to CurveStart");
            MovePointExp(FRI,CurveStart,5,3.5,0.001,0.15); // Move to begin Passage
            break;
        case 1:
    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

					                FORCE CONTORL 

    1. Find the net contact forces at the TCP
    2. Insert the most recent into a buffer (stack)
    3. Low pass filter the force data to limit noisy measurements

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
	

        ResolveForceatTCP( bMt, eMf, eMt,Measuredforce,ResolvedForce,x);
        fifoBufferInsert(ForceBuffer,ResolvedForce);
        LowPassFilter(ForceBuffer,MovingAverageForce);


        printf("\n Resolved Force=");printfM(ResolvedForce);
        printf("\n MovingAverageForce Force=%f",MovingAverageForce[1]);

 

    /*------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                    Actually I need to follow a trajectory or else it will stop

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */

        printf("Cutting time=%f \n",tcut); 
        tcut+=cycletime;
        if(tcut>tfcut)
	    {
	    tcut=tfcut;        
        XTrajecSim(bMtinit,bMtFinal,bMtd,tcut,tfcut,bVt,bOmegat);
            for(int i=0;i<3;i++)
            {
            bVt[i]=0.0;bOmegat[i]=0.0;
            }
	    }
        else
        {   
        XTrajecSim(bMtinit,bMtFinal,bMtd,tcut,tfcut,bVt,bOmegat);
        }
   

        printf("Velocity from trajectory =\n");printfM(bVt);
        printf("\nbMt=\n");printfM(bMt);        
        printf("\nbMtd=\n");printfM(bMtd);

        printfM(bVt);

        bMtd.extract(bPtd);  bMtd.extract(bRtd); 

        for(int i=0;i<3;i++)
        {
        
        for(int j=0;j<3;j++)
        {
        bRtdT[i][j]=bRtd[j][i];
        }
        }

        R_S=bRt*bRtdT; 
 
        Sthetau.buildFrom(R_S);//theta u representation
        
        for (int i = 0; i < 3; i += 1)
        {
        T_S[i]=0.5*(bPtd[i]-bPt[i])+0.045*(bVt[i]);
        Sthetau[i]=0.2*Sthetau[i]+0.01*(bOmegat[i]) ;  
        }    
        

        T_S[2]=0.0;
   


        printf("\n Velocity pre deviation =\n");printfM(T_S);


    /*************************************************************************************************************

    Calculate the joint velocity

    *************************************************************************************************************/
  
    
    for(int i=0;i<3;i++)
    {	
       Vbase[i]=LambdaCart[i]*T_S[i];
       Vbase[i+3]=LambdaCart[i+3]*Sthetau[i];
    } 

    RotateScrew(Vbase,tV,tRb);


    printf("\n V in base frame");printfM(Vbase);
    printf("\n V in tool frame");printfM(tV);
    

     if(MovingAverageForce[1]<-0.2) // Try to filter noise
     {
        tV[2]+=(SliceGain*MovingAverageForce[1]); // slicing motion 
     }
     printf("Velocity post deviation =\n");printfM(tV);

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
	
    break;
    }// End the if block





    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    

	// Writing Data to file

    for(int i=0;i<3;i++)
    {
        OutputS<< T_S[i] << " ";  //Cart Velocity base
    }
    
    for(int i=0;i<3;i++)
    {
        OutputS<< Sthetau[i] << " ";  //Cart Angular Velocity base
    }
    OutputS << endl;
        

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
		OutputMeasuredbMtd<< bMtd[i][0] << " ,"; 
		OutputMeasuredbMtd<< bMtd[i][1] << " ,"; 
		OutputMeasuredbMtd<< bMtd[i][2] << ", "; 
		OutputMeasuredbMtd<< bMtd[i][3] << " "; 
		OutputMeasuredbMtd<<endl;
	}
    // Measured Force raw from force sensor


    for(int i=0;i<6;i++)
    {
        OutputCmdtV<< tV[i] << " ";  //Velocity in tool frame
    }

    for(int i=0;i<6;i++)
    {
    OutputMeasuredForce<< Measuredforce[i] << " ";  //save Force Sensor
    }
    OutputMeasuredForce << endl;

    for(int i=0;i<6;i++)
    {
    OutputResolvedForce<< ResolvedForce[i] << " ";  //save Force Sensor
    }
    OutputResolvedForce << endl;
  

    for(int i=0;i<6;i++)
    {
        OutputMovingAverageForce<< MovingAverageForce[i] << " ";  //save Force Sensor
    }
    OutputMovingAverageForce << endl;
  

    printf("\n time Counter=%f \n",timecounter);
    printf("\n-----------------------------------------------------------------------\n");


  }
  




	FRI->StopRobot();	
	OutputS.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputResolvedForce.close();
	OutputMovingAverageForce.close();
}





















