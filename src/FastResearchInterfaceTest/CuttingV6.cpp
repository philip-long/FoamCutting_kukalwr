
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


Moving in a curve 


**************************************************************** */


void CuttingV6(FastResearchInterface *FRI)
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
float MeasuredPose[12];
float timecounter; // actual time spent
double TotalTime;
float MeasuredJointValuesInRad[7];
float JointValuesInRad[7];
cycletime = FRI->GetFRICycleTime();

/*-------------------------------------------------------------------------------------------------------

ViSP VARIABLES

-------------------------------------------------------------------------------------------------------*/

vpColVector bVt(6),V(6),tV(6),bOmegat(3),qdot(6),qinit(7),qt(7);


vpMatrix J(6,7); // Jacobian Matrix


vpRotationMatrix bRt,cutRt,tRb;

vpHomogeneousMatrix bMtinit,bMtFinal,bMt,tMb;



/*-------------------------------------------------------------------------------------------------------

Polynomial and curve parameters

-------------------------------------------------------------------------------------------------------*/
vpColVector CurveParameters(5);
float p[6]; // Polynomial
int PolyDegree=2; // Order of the polynomial
p[0]=-2.1544;p[1]=-5.6970;p[2]=-3.7439;p[3]=0.0;p[4]=0.0;p[5]=0.0;
p[0]=-1.1167;p[1]=-3.5970;p[2]=-2.9083 ;p[3]=0.0;p[4]=0.0;p[5]=0.0;
 
printf("Polynomial =%fx^2+%fx+%fx", p[2],p[1],-p[0]);
/*-------------------------------------------------------------------------------------------------------

INPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
vpHomogeneousMatrix cMt,tMc,eMf,eMt,cutMt,bMtd;
double Kf[7];  // Force gain
double Kfi[6];
float temp_x[11]; // A temporary variable 
double LambdaCart[6];
double Var1;
double Var2;
double DesiredForce[6]; // Superimposed force
double Kdeltaf; // Gain multiplied by force error
 GETHomogeneousMatrix(tMc,"./ConstantMatrices/tMc");  //Camera to Tool frame
GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame
printf("cutMt=\n");printfM(cutMt);
GETParametersCI("./CuttingV6/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
GETParametersCI("./CuttingV6/Kf", Kf, Var1); //Using these parameters as misc for debug
GETParametersCI("./CuttingV6/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
cutMt.extract(cutRt);

/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./CuttingV6/IdentifiedForceParameter","rb");
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
	

for (int i=0;i<11;i++)
{
x[i]=temp_x[i];
}


printf("\n X=%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[9]);


/*-------------------------------------------------------------------------------------------------------

OUTPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
 

   

    ofstream OutputS;     OutputS.open("./CuttingV6/Results/S",std::ios::out | std::ios::app );
    ofstream OutputCurveParameters;     OutputCurveParameters.open("./CuttingV6/Results/Curve",std::ios::out | std::ios::app );
    ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingV6/Results/bMt",std::ios::out | std::ios::app );
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingV6/Results/MeasuredForce");
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingV6/Results/ResolvedForce");
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingV6/Results/MovingAverageForce");

//*************************************END OF DECLARATIONS**********************************************************************



// Get initial positions
FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);

float CutDepth=0.2;
bMtFinal=bMtinit;
bMtFinal[0][3]=-0.45;
printf("\n bMtinit=\n");printfM(bMtinit);
printf("\n bMtFinal=\n");printfM(bMtFinal);


printf("TotalTime  =%f\n",TotalTime);
/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/
  while((FRI->IsMachineOK()) && timecounter<TotalTime)
  {
    printf("Counter=%f\n",timecounter);
  	timecounter+=cycletime;
	FRI->WaitForKRCTick();

    /*************************************************************************************************************

    Get the data from the FRI

    *************************************************************************************************************/

    FRI->GetCurrentJacobianMatrix(FriJaco);FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame

	FRI->GetMeasuredCartPose(MeasuredPose);    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);

    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);

	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

    bMt.extract(bRt);    tMb=bMt.inverse();tMb.extract(tRb);
    printf("\n bMt=\n");printfM(bMt);
    /*************************************************************************************************************

    Generates a Curve and a velocity
    *************************************************************************************************************/
    printf("t=%f",timecounter);    printf("tf=%f",TotalTime);


    CurveGen(p,PolyDegree,timecounter,TotalTime,bMtinit,bMtFinal,CurveParameters);
    printf("\n Xt=%f,Xdott=%f,Yt=%f,Ydot=%f,theta=%f",CurveParameters[0],CurveParameters[1],CurveParameters[2],CurveParameters[3],CurveParameters[4]);
    CurveVelocity(bMt,CutDepth,CurveParameters,bMtd);

    printf("\n Desired Frame=\n");printfM(bMtd);
    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

					                FORCE CONTORL 

    1. Find the net contact forces at the TCP
    2. Insert the most recent into a buffer (stack)
    3. Low pass filter the force data to limit noisy measurements

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
	


    ResolveForceatTCP( bMt, eMf, eMt,Measuredforce,ResolvedForce,x);
    // Have to change the frame now to the cutting frame quite disasterously

    fifoBufferInsert(ForceBuffer,ResolvedForce);
    LowPassFilter(ForceBuffer,MovingAverageForce);

    printf("\n Resolved Force=");printfM(ResolvedForce);
    printf("\n Resolved Force in tool frame=");printfM(ResolvedForce2);
    
    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

					                VELOCITY CONTROL
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
    vpTranslationVector bPtd,bPt,T_S;
    vpRotationMatrix bRtd,bRt,bRtdT,R_S;
    vpThetaUVector Sthetau;

    bMtd.extract(bPtd);    bMt.extract(bPt);
    bMtd.extract(bRtd);    bMt.extract(bRt);
    
    T_S=bPtd-bPt;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            bRtdT[i][j]=bRtd[j][i];
        }
    }

    R_S=bRt*bRtdT;
    printf("Diff Mat=\n");printfM(R_S);    
    Sthetau.buildFrom(R_S);//theta u representation
    printf("Error Position=\n");printfM(T_S);
    printf("Error Orientation=\n");printfM(Sthetau);
        
    
    for(int i=0;i<3;i++)
    {	
        bVt[i]=(0.5*T_S[i]);
        bVt[i+3]=(-0.3*Sthetau[i]);
    } 

    printf("\n V in base frame");printfM(bVt);

    RotateScrew(bVt,tV,tRb);

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
        OutputCurveParameters<< CurveParameters[i] << " ";  //save cart pose
    }
   OutputCurveParameters<< endl;
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





















