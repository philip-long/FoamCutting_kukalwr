
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


Cutting in a curve PBVS Actual Cutting this time
    1. Move to an initial position at the curve begining
    2. Begin to follow curved
    3. if there is a resistive force begin slicing motion
    4. if we have exited media return to curve start
    5. In this version we try to use a local update from vision!!

**************************************************************** */


void CuttingV12(FastResearchInterface *FRI)
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
float timecounter=0.0; // actual time spent
double TotalTime;
float MeasuredJointValuesInRad[7];
float JointValuesInRad[7];
cycletime = FRI->GetFRICycleTime();

/*-------------------------------------------------------------------------------------------------------

ViSP VARIABLES

-------------------------------------------------------------------------------------------------------*/

// Create the variables requried for the camera
bool reset = true; // Enable bus reset during construction (default)
vp1394TwoGrabber g(reset);
if (reset) 
{
vpTime::wait(50); // Wait 1000 ms
}
g.setCamera(0);
g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
vpCameraParameters camdistored;

// Camera Parameters updated 5-Nov-2013
 camdistored.initPersProjWithDistortion(1267.612068,1268.174369, 320.344917, 236.6983837,-0.1277547589,0.129434146);



//cam.initPersProjWithoutDistortion(468.24,467.73,371.5633,263.027243);
// Create a ViSP gray image
vpImage<unsigned char> I;
IplImage* original = 0;
IplImage* displayedImage = 0;
float VisErrPos;

vpThetaUVector VisErrRot;

double xn1,yn1,xn2,yn2,xn3,yn3;
double xn4,yn4,xn5,yn5,xn6,yn6;
float uk,vk;
float uk1,uk2,uk3,uk4,uk5,uk6;
float vk1,vk2,vk3,vk4,vk5,vk6;;


int TotalPoints;

uk=287.0;//vk=395.0;
//vk=440.0;
vk=395.0;

vpColVector deltas(2);
vpColVector cVcam(6),tVcam(6);
vpMatrix Lpts(2,6);
float CollineationParams[6];
CollineationParams[0]=468.2421; // fu 
CollineationParams[1]=467.7332; // fv
CollineationParams[2]=0.0; // fs
CollineationParams[3]=371.5633; // u0
CollineationParams[4]=253.78080; // v0

//    unsigned int ncameras = 0;
vpColVector ublob(100);
vpColVector vblob(100);
int TrackBars[7];


// Init filter
TrackBars[0]=81;TrackBars[1]=360;TrackBars[2]=45;TrackBars[3]=40;TrackBars[6]=100;

float JointSpeedGain=0.0;

cvNamedWindow("Blob Extraction", 0);
cvCreateTrackbar( "Grayscale Threshold", "Blob Extraction", &TrackBars[0], 255, NULL );
cvCreateTrackbar( "Maximum Blob Size", "Blob Extraction", &TrackBars[1], 10000, NULL );// Maximum blob size, these are functions of the knife
cvCreateTrackbar( "Minimum Blob Size", "Blob Extraction", &TrackBars[2], 4000, NULL );// Manimum blob size
cvCreateTrackbar( "Aspect ratio", "Blob Extraction", &TrackBars[3], 100, NULL );// Aspect ratio best is 1
cvCreateTrackbar( "Cursor Move u","Blob Extraction", &TrackBars[4], 1000, NULL );
cvCreateTrackbar( "Cursor Move v","Blob Extraction", &TrackBars[5], 1000, NULL );
cvCreateTrackbar( "Cutting Speed","Blob Extraction", &TrackBars[6], 100, NULL );

//***************************VISP subsection declarations***************************
// Vectors and Matrices
vpColVector bVt(6),V(6),tV(6),bOmegat(3),qdot(6),qinit(7),qt(7);
vpMatrix J(6,7); // Jacobian Matrix
vpRotationMatrix bRt,tRb,tRcam;
vpHomogeneousMatrix bMtinit,bMtd,bMt,tMb;
vpColVector camPim(4),bPim(4);


/*-------------------------------------------------------------------------------------------------------

Polynomial and Cutting parameters

-------------------------------------------------------------------------------------------------------*/
vpColVector CurveParameters(5);
float p[6]; // Polynomial
int PolyDegree=2; // Order of the polynomial
   
p[0]=1.238428 ; p[1]=3.579001 ; p[2]=3.229153;p[3]=0.0;p[4]=0.0;p[5]=0.0;      
p[0]=1.288428 ; p[1]=3.579001 ; p[2]=3.229153; 
//p[0]=1.238428 ; p[1]=3.579001 ; p[2]=3.229153;

p[0]=-1.085184 ; p[1]=-5.462084 ; p[2]=-4.955168;

printf("Polynomial =(%f)x^2+(%f)x+(%f)", p[2],p[1],p[0]);
//tfcut=30;
float tfcut=40.2;

float tcut=0.0;
int Cutting=0;
vpTranslationVector bPtd,bPt,T_S;
vpHomogeneousMatrix CurveStart,CurveatPim;
vpRotationMatrix bRtd,bRtdT,R_S,R_Sv;
vpThetaUVector Sthetau;

/*

Load the trajectory that has been calculated offline

*/

printf("\ntfcut=%f,cycletime=%f",tfcut,cycletime);
int NbrofTrajectpts=(int)( (tfcut+0.001+cycletime)/(cycletime)); 
printf("\n NbrofTrajectpts=%d",NbrofTrajectpts);
vpMatrix Trajectory(NbrofTrajectpts,3);
loadtrajectory(FRI,Trajectory,NbrofTrajectpts,"./CuttingV12/CurveTrajectory");
printf("\nTrajecory=\n");printfM(Trajectory);
int trajpoint=0;



//==========================================================================================//



/*-------------------------------------------------------------------------------------------------------

INPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv;
vpRotationMatrix bRtv,bRtvT,bRcurveim;
double Kf[7];  // Force gain
double Kfi[6];
float temp_x[11]; // A temporary variable 
double LambdaCart[6];
double Var1;
double Var2;
double DesiredForce[6]; // Superimposed force
double Kdeltaf; // Gain multiplied by force error
GETHomogeneousMatrix(tMcam,"./ConstantMatrices/tMc");  //Camera to Tool frame
GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame
GETParametersCI("./CuttingV12/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
GETParametersCI("./CuttingV12/Kf", Kf, Var1); //Using these parameters as misc for debug
GETParametersCI("./CuttingV12/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
printf("\n tMcam=\n");printfM(tMcam);
camMt=tMcam.inverse();
tMcam.extract(tRcam);
printf("camMt=\n");printfM(tMcam);


/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./CuttingV12/IdentifiedForceParameter","rb");
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
  

    ofstream OutputS;     OutputS.open("./CuttingV12/Results/S",std::ios::out | std::ios::trunc );
    ofstream Outputuv;     Outputuv.open("./CuttingV12/Results/Uv",std::ios::out | std::ios::trunc );
    ofstream OutputDS;    OutputDS.open("./CuttingV12/Results/Sdelta",std::ios::out | std::ios::trunc );
    ofstream OutputCurveParameters;     OutputCurveParameters.open("./CuttingV12/Results/Curve",std::ios::out | std::ios::trunc );
    ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingV12/Results/bMt",std::ios::out | std::ios::trunc  );
    ofstream OutputCmdbV;      OutputCmdbV.open("./CuttingV12/Results/OutputCmdbV",std::ios::out | std::ios::trunc  );
    ofstream OutputCmdtV;      OutputCmdtV.open("./CuttingV12/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingV12/Results/MeasuredForce",std::ios::out | std::ios::trunc );
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingV12/Results/ResolvedForce",std::ios::out | std::ios::trunc );
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingV12/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

//*************************************END OF DECLARATIONS**********************************************************************










/*************************************************************************************************************

Initialising the robot

*************************************************************************************************************/
FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);


printf("\n bMtinit=\n");printfM(bMtinit);


// Eventually both of these will be supplied by camera
float MaterialHeight=.110; 
//float MaterialHeight=.135; 
float CutDepth=.09; 
//CutDepth=.08; 
printf("TotalTime  =%f\n",TotalTime);
 printf("cycletime=%f\n",cycletime);

/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/
vpHomogeneousMatrix bMtFinal;
bMtFinal[0][3]=-0.460;


// Move to a starting position defined as the given position with the curve orientation
CurveEval(p,PolyDegree,bMtinit,CurveStart);

// Cutting angle stuff
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Y is aligned to curve tangent => X is cutting angle
float CuttingAngle=PI/9;
vpHomogeneousMatrix tMtc;
TransMat(1,CuttingAngle,tMtc);
CurveStart[2][3]=CutDepth;
CurveStart=CurveStart*tMtc;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
float DepthZ;
DepthZ=camMt[2][3]-0.015;
MovePointExp(FRI,CurveStart,2,3,0.005,0.1); // Move to begin Passage
bMtsafe=CurveStart;
bMtsafe[2][3]=.15; // a safe position outside the meat but with same orientation as curve start to save time
// 


  while((FRI->IsMachineOK()))
  {
   

    FRI->WaitForKRCTick();


    // Ensure the vision deviations begin at zero
    VisErrRot[0]=0.0;VisErrRot[1]=0.0;VisErrRot[2]=0.0;VisErrPos=0.0;

  	timecounter+=cycletime;
	
   

    /*************************************************************************************************************

    Get the data from the FRI

    *************************************************************************************************************/

    FRI->GetCurrentJacobianMatrix(FriJaco);FRIJaco2vpMatrix(FriJaco,J); 

	FRI->GetMeasuredCartPose(MeasuredPose);    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);

    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);

	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);

    bMt.extract(bRt);    tMb=bMt.inverse();tMb.extract(tRb);

   // printf("bMt=\n");printfM(bMt);

    /*************************************************************************************************************

    Generates a Curve and a velocity
    *************************************************************************************************************/
   
    if(bMt[2][3]>MaterialHeight || bMt[0][3]>-0.445) // if outside body or in reset mode  
    {
    Cutting=0;
    tcut=0;
    trajpoint=0;
    }
    else
    {
    Cutting=1;
    }


    switch (Cutting){
        case 0: // Not cutting

            // 1. End the camera acquisition
            cvReleaseImage( &original );
            cvReleaseImage( &displayedImage );
            //cvDestroyWindow(  "Blob Extraction" );
            //cvDestroyWindow( "OpenCv show" );
            if ( (cvWaitKey(10) & 255) == 27 ) break;

            printf(" Exceeded maximum height"); 
            printf("\n Z is %f,while X is %f",bMt[2][3],bMt[0][3]);
            printf("\n\n\nRESETING \n \n \n");
            printf("\n Move to Safe Point bMtsafe2");
            // 2. Reset point
            bMtsafe2=bMt;
            bMtsafe2[2][3]=bMt[2][3]+0.035;
            MovePointExp(FRI,bMtsafe2,7.0,1,0.01,0.5); // Move to safe point
            printf("\n Move to Safe Point bMtsafe");
            MovePointExp(FRI,bMtsafe,2.0,1,0.01,0.4); // Move to safe point
            printf("\n Move to CurveStart");
            MovePointExp(FRI,CurveStart,5,3.5,0.002,0.15); // Move to begin Passage

            break;
        case 1:
           /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                          CURVE TRAJECTORY
     ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

            
            printf("Cutting time=%f",tcut); 
            tcut+=cycletime;
            trajpoint+=1;
            if(trajpoint>NbrofTrajectpts)trajpoint=NbrofTrajectpts;
            CurveParameters[0]=Trajectory[trajpoint][0];
            CurveParameters[1]=0;
            CurveParameters[2]=Trajectory[trajpoint][1];
            CurveParameters[3]=0;
            CurveParameters[4]=Trajectory[trajpoint][2];

            CurveVelocity(bMt,CutDepth,CurveParameters,bMtd); // note this is an overloaded function
                      
            bMtd=bMtd*tMtc;
            bMtd.extract(bPtd);    bMt.extract(bPt);
            bMtd.extract(bRtd);    bMt.extract(bRt);
            

           /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VISION CONTORL 

            1. Acquire Image and remove distortion
            2. Extract the position of the blobs
            3. Create the local deviation to the knife
      ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

            DepthZ=camMt[2][3]+(bMt[2][3]-MaterialHeight);
            printf("\n Estimated Cutting Depth =%f \n",DepthZ);

            g.acquire(I); 
            vpImageConvert::convert(I,original);

            if(!displayedImage)   displayedImage = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,3);
            

            // Find all the blobs in image
            ImagePointFind(original,displayedImage,TrackBars,ublob,vblob,TotalPoints); 

            // Get points, [uk1,vk1] is the point nearest the knife
            //             [uk2,vk2] is the point nearest [uk1,vk1]
            //             [uk3,vk3] is the point nearest [uk2,vk2]
    
            SortImageVector(uk,vk,uk1,vk1,ublob,vblob,TotalPoints);
            SortImageVector(uk1,vk1,uk2,vk2,ublob,vblob,TotalPoints);
            SortImageVector(uk2,vk2,uk3,vk3,ublob,vblob,TotalPoints);
            SortImageVector(uk2,vk2,uk3,vk3,ublob,vblob,TotalPoints);
            SortImageVector(uk3,vk3,uk4,vk4,ublob,vblob,TotalPoints);
            SortImageVector(uk4,vk4,uk5,vk5,ublob,vblob,TotalPoints);
            SortImageVector(uk5,vk5,uk6,vk6,ublob,vblob,TotalPoints);
            // Shift all pixels to the right

            uk1=uk1-50;uk2=uk2-50;uk3=uk3-50;
            uk4=uk4-50;uk5=uk5-50;uk6=uk6-50;

            // Display

            cvCircle(displayedImage,cvPoint(uk,vk),2,CV_RGB(0, 255,255), -1,8,0);
/*
            cvCircle(displayedImage,cvPoint(uk1,vk1),3,CV_RGB(0, 0, 255), -1,8,0);
            cvCircle(displayedImage,cvPoint(uk2,vk2),3,CV_RGB(0, 255, 0), -1,8,0);
            cvCircle(displayedImage,cvPoint(uk3,vk3),3,CV_RGB(100, 0,100 ), -1,8,0);
            cvCircle(displayedImage,cvPoint(uk4,vk4),3,CV_RGB(0, 0, 255), -1,8,0);
            cvCircle(displayedImage,cvPoint(uk5,vk5),3,CV_RGB(0, 255, 0), -1,8,0);
            cvCircle(displayedImage,cvPoint(uk6,vk6),3,CV_RGB(100, 0,100 ), -1,8,0);
*/
            cvShowImage( "Blob Extraction", displayedImage );
            
            // Convert ukd, vkd to a 3D coorindate using knife depth
            vpPixelMeterConversion::convertPoint(camdistored,uk1,vk1,xn1,yn1);
            vpPixelMeterConversion::convertPoint(camdistored,uk2,vk3,xn2,yn2);
            vpPixelMeterConversion::convertPoint(camdistored,uk3,vk3,xn3,yn3);
            vpPixelMeterConversion::convertPoint(camdistored,uk4,vk4,xn4,yn4);
            vpPixelMeterConversion::convertPoint(camdistored,uk5,vk5,xn5,yn5);
            vpPixelMeterConversion::convertPoint(camdistored,uk6,vk6,xn6,yn6);
            // Create a desired frame bMtv purely from local vision            
            visionFramefrom3Pts(xn1,yn1,xn2,yn2,xn3,yn3,tMcam,bMt,bMtv,DepthZ);         
            bMtv=bMtv*tMtc;
           // printf("Frame using 3pts Vision \n ");printfM(bMtv);   
         //   visionFramefrom6Pts(xn1,yn1,xn2,yn2,xn3,yn3,xn4,yn4,xn5,yn5,xn6,yn6,tMcam,bMt,bMtv,DepthZ); 
           // bMtv=bMtv*tMtc;
           // printf("Frame using 6pts Vision \n ");printfM(bMtv);   

            // Evaluate the estimated frame at the vision point 
            CurveEval(p,PolyDegree,bMtv,CurveatPim);    
            CurveatPim=CurveatPim*tMtc;
            printf("\n  Using Curve Eval to find at image point \n");printfM(CurveatPim);        
           printf("Current Position \n ");printfM(bMt);  
        //    printf("\n  Desired from Trajectory \n");printfM(bMtd);        
            
            // Find the errors between the frames
            bMtv.extract(bRtv);    CurveatPim.extract(bRcurveim);

            TranposeRotMat(bRtvT,bRtv);


            R_Sv=bRcurveim*bRtvT;
        //    printf("Diff Mat=\n");printfM(R_Sv);    
            

            // This is the error in the Y direction
            VisErrPos=CurveatPim[1][3]-bMtv[1][3];
            VisErrRot.buildFrom(R_Sv);//theta u representation
            
            // Need to check if either of these contain a NaN
            for (int i = 0; i < 3; i += 1)
            {
                if( isnan(VisErrPos))VisErrPos=0;
                if( isnan(VisErrRot[i]))VisErrRot[i]=0;
            }
    
            // This is the error in orientation
            
            if ( (cvWaitKey(10) & 255) == 27 ) break;

            /*------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            FORCE CONTORL 

            1. Find the net contact forces at the TCP
            2. Insert the most recent into a buffer (stack)
            3. Low pass filter the force data to limit noisy measurements

            ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */


            ResolveForceatTCP( bMt, eMf, eMt,Measuredforce,ResolvedForce,x);
            fifoBufferInsert(ForceBuffer,ResolvedForce);
            LowPassFilter(ForceBuffer,MovingAverageForce);

            printf("\n ResolvedForce in tool frame=");printfM(ResolvedForce);


            /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VELOCITY CONTROL

            ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

            T_S=bPt-bPtd;
            TranposeRotMat(bRtdT,bRtd);

            R_S=bRt*bRtdT;
            //printf("Diff Mat=\n");printfM(R_S);    
            Sthetau.buildFrom(R_S);//theta u representation
            printf("Error Position=\n");printfM(T_S);
            printf("Error Orientation=\n");printfM(Sthetau);
//            printf("Position vision=[%f,%f,%f,]\n",bMtv[0][3],bMtv[1][3],bMtv[2][3]);
            printf("Error vision Position=%f \n",VisErrPos);
            printf("Error Orientation vision=\n");printfM(VisErrRot);
            // The desired velocity is modified by the vision term
              T_S[1]+=VisErrPos;    //VISION ERROR UNCOMMENT
           // Eliminating the sawing motion by preventing the position from moving knife deeper into the material
           // However there has to be some gain otherwise it tends to drift
            T_S[2]=0.0;

         //   printf("\n T_S");printfM(T_S);


            for(int i=0;i<3;i++)
            {	
 //            bVt[i]=(0.8*T_S[i]);
//             bVt[i+3]=(-0.4*Sthetau[i]);
               bVt[i]=(-3.0*T_S[i]);
               bVt[i+3]=(-0.8*Sthetau[i]);
               bVt[i+3]=-0.8*(Sthetau[i]+0.5*VisErrRot[i]);  //VISION ERROR UNCOMMENT
            } 
            

            printf("\n V in base frame");printfM(bVt);
            
            RotateScrew(bVt,tV,tRb);
            printf("\n V in tool frame");printfM(tV);


            if(MovingAverageForce[1]<-0.4) // Try to filter noise
            {
            //tV[2]+=(0.0075*MovingAverageForce[1]);
            tV[2]+=(0.006*MovingAverageForce[1]);
            }
            
                       
            qdot=J.pseudoInverse()*tV;
    

            printf("\n qdot sent to robot");printfM(qdot);

            /*************************************************************************************************************

            Send to FRI

            *************************************************************************************************************/


            JointSpeedGain=0.0;
            JointSpeedGain=(float) TrackBars[6]/100; // Just a final security measure so we can regulate the speed online
           // printf("TrackBars[6]=%d,JointSpeedGain=%f",TrackBars[6],JointSpeedGain);
            for(int i=0;i<7;i++)
            {
                dq[i]=JointSpeedGain*qdot[i];
                JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (dq[i]*cycletime);
            //    JointValuesInRad[i]=qt[i];
            } 

            FRI->SetCommandedJointPositions(JointValuesInRad);


    
            break;
    }
    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    

	// Writing Data to file
	    for(int i=0;i<4;i++)
	    {
		    Outputuv<< bMtv[i][0] << " ,"; 
		    Outputuv<< bMtv[i][1] << " ,"; 
		    Outputuv<< bMtv[i][2] << ", "; 
		    Outputuv<< bMtv[i][3] << " "; 
		    Outputuv<<endl;
	    }

        OutputDS<< deltas[0] << " ";  //save cart pose
        OutputDS<< deltas[1] << " ";  //save cart pose
        OutputDS << endl;

        for(int i=0;i<6;i++)
        {
            OutputCmdtV<< tV[i] << " ";  //save cart pose
            OutputCmdbV<< bVt[i] << " ";  //save cart pose
        }
        OutputCmdtV<< endl;
        OutputCmdbV<< endl;

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
    printf("=======================================================================\n\n");
  }

    cvReleaseImage( &original );
    cvReleaseImage( &displayedImage );
    cvDestroyWindow(  "Blob Extraction" );
    cvDestroyWindow( "OpenCv show" );

	FRI->StopRobot();	
	OutputS.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputResolvedForce.close();
	OutputMovingAverageForce.close();
    Outputuv.close();
}





















