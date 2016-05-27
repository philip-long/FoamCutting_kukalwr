#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/CuttingV15.h>



void FunctionTest(FastResearchInterface *FRI)
{	

/*-------------------------------------------------------------------------------------------------------

FRI Variables

-------------------------------------------------------------------------------------------------------*/

float CommandedJointVelocity[7]; // desired joint velocity
float MeasuredJointValues[7]; // Measure joint values
float CommandedJointValues[7];// Desired joint values
float cycletime;// cycle time
float Measuredforce[6]; // measured force
float MeasuredPose[12];// measured tool 
float timecounter=0.0; // actual time spent
float **FriJaco;  //to store the measured Jacobian from KUKA
FriJaco = new float* [6];
for(int i=0;i<6;i++)
FriJaco[i] = new float[7];

vpHomogeneousMatrix bMtinit,bMtd,bMt,tMb,bMcamd,bMcam;
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv;
float temp_x[11]; // A temporary variable 
GETHomogeneousMatrix(tMcam,"./ConstantMatrices/tMc");  //Camera to Tool frame
GETHomogeneousMatrix(bMcamd,"./ConstantMatrices/bMcamd");  //Camera to Tool frame
GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame

printf("\n tMcam=\n");printfM(tMcam);
camMt=tMcam.inverse();
printf("camMt=\n");printfM(tMcam);

FRI->GetMeasuredJointPositions(MeasuredJointValues);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);
printf("\n Initial Frame");printfM(bMtinit);
bMcam=bMtinit*tMcam;
bMcamd[0][3]=bMcam[0][3];bMcamd[1][3]=bMcam[1][3];bMcamd[2][3]=bMcam[2][3];
bMtd=bMcamd*camMt;

printf("\n Desired Frame");printfM(bMtd);
MovePointExp(FRI,bMtd,2,3,0.005,0.1);


}
