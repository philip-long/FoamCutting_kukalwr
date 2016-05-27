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
// ViSP includes
#include <visp/vpVideoWriter.h>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpPose.h>
#include <visp/vpMath.h>
#include <visp/vpImageConvert.h>
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
#include <visp/vpMeterPixelConversion.h>
#include<visp/vpPixelMeterConversion.h>
#include<visp/vpDisplayOpenCV.h>
// OpenCv includes
#include "cv.h" 
#include "highgui.h" 
#include <stdio.h>  
#include "cvblobs/BlobResult.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
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

This is the first version of the new wave of tests. 

ForkV2: Using the fork program as a basis and we aim to move the kuka robot to position the fork dots to a desired image 

In this particular case, we are not using ViSP, we are using 4 dots

Objectives is to make sure the camera frames are good and basically setup up the schem


*/
void ForkV2(FastResearchInterface *FRI)
{


    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
 printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
 printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
 printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
 printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
 printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    /*
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------	

    Declare  All Variables  

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */

    //************************************* FRI VARIABLES/CONTROLLER VARIABLES**********************************************
    double timecounter=0;
    float MeasuredPose[12];
    float JointStiffnessValues[7];
    float JointDampingValues[7]; 
    float JointValuesInRad[7];  // to store the joint value
    float dq[7];
    float MeasuredJointValuesInRad[7];
    float Measuredforce[6];
    
    // FRI 	Variables
    float cycletime;
    int ResultValue = 0;   
    int Detect=1;
    int ControlStage=0;
    float CuttingSpeed;
    //*****************************************************************************************************************************

    //************************************* OPEN CV VARIABLES**************************************************************
    char wndname[] = "Blob Extraction";
    char tbarname1[] = "Grayscale Threshold";
    char tbarname2[] = "Maximum Blob Size";
    char tbarname3[] = "Minimum Blob Size";
    char tbarname4[] = "Aspect ratio";
    int param1,param2,param3,param4,CursorU,CursorV,CuttingVelocity;
    CuttingVelocity=100; // Default meaning 1

    IplImage* originalThr = 0;
    IplImage* original = 0;
    IplImage* displayedImage = 0;


    float Xc1[2];
    float Xc2[2];
    float Yc1[2];
    float Yc2[2];
    int ExistingFeature=0;




    // Blobs
    int IndexofBlob[4];
    float Xcentre[4];
    float Ycentre[4];
    float DepthZ[4];
    float CollineationParams[6];
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;


    //**********************************************************************************************************************

    //************************************* VISP VARIABLES*****************************************************************
    vpMatrix Lpts1(4,6);
    vpMatrix Lpts2(4,6);
    vpMatrix L4pts(8,6);
    vpColVector deltas(8);
    vpColVector deltasg(8);
    vpColVector sdesired(8);
    vpColVector bPblob(4);
    vpColVector cPblob(4);
    vpColVector s(8);
    for(int i=0;i<8;i++)
        s[i]=0;

    vpColVector FOVc1(4);
    vpColVector FOVc2(4);
    for(int i=0;i<4;i++)
    {
        FOVc1[i]=0.0;
        FOVc2[i]=0.0;
    }

    vpColVector V_cam(6);
    vpColVector qdot(7);
    vpColVector V(6); // Kinematic Screw of tool frame
    vpMatrix J(6,7); // Jacobian Matrix
    vpHomogeneousMatrix bMc,cMb,bMt,cMt,tMc,eMf,eMt;


    //initialize Jacobian matrix, 6*7, all 0
    float **FriJaco;  //to store the measured Jacobian from KUKA
    FriJaco = new float* [6];
    for(int i=0;i<6;i++)
    FriJaco[i] = new float[7];
    //***********************************************************************************************************************


    //************************************* CAMERA VARIABLES**************************************************************
    param1 = 24; // Initialise Parameters, Param one is the grayscale level
    param2 = 2000;// Param two is Maximum blob size
    param3 = 400;// Param three is Mimimum blob size
    param4=80;// Param four is aspect ratio


    // Estimation of depth, we have taken an offline measurement of the position of the blobs this is used to estimate the depth at each iteration

    bPblob[0]=-0.4; 
    bPblob[1]=0.0;
    bPblob[2]=0.05; // Estimation of depth
    bPblob[3]=1.0;

    CollineationParams[0]=592.4859071; // fu 
    CollineationParams[1]=590.8824455; // fv
    CollineationParams[2]=0.0; // fs
    CollineationParams[3]=374.4020879; // u0
    CollineationParams[4]=180.2622981; // v0
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );

    if ( !capture ) 
    {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    }

    // Create a window in which the captured images will be presented
    cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );   
    cvNamedWindow(wndname, 0);
    cvCreateTrackbar( tbarname1, wndname, &param1, 255, NULL );
    cvCreateTrackbar( tbarname2, wndname, &param2, 6000, NULL );// Maximum blob size, these are functions of the knife
    cvCreateTrackbar( tbarname3, wndname, &param3, 4000, NULL );// Minimum blob size
    cvCreateTrackbar( tbarname4, wndname, &param4, 100, NULL );// Aspect ratio best is 1
    cvCreateTrackbar( "Cursor Move u",wndname, &CursorU, 1000, NULL );
    cvCreateTrackbar( "Cursor Move v",wndname, &CursorV, 1000, NULL );
    cvCreateTrackbar( "Cutting Velocity",wndname, &CuttingVelocity, 100, NULL );



    //*****************************************************************************************************************************


    //************************************* INPUT VARIABLES**********************************************************************


    // Parameters taken from file, can change to save data with different suffixs
    double Kf[7];  // Force gain
    double Kfi[6];
    double TotalTime;
    double LambdaCart[6]; // Cartesian Stiffness
    double DesiredForce[6]; // Superimposed force
    double Kdeltaf; // Gain multiplied by force error
    double Var1;
    double Var2;
    float sdesired1[8];
    float lambdas[8];
    float sdesired2[8];
    float temp_x[11]; // A temporary variable 

    FILE * pFile;pFile=fopen("/home/kuka/Documents/PhilipLong/FRILibrary/MinForceSensor/Linux/x86/debug/bin/ForkV2/lambdas","rb");
    fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7]);
    //x[0]=8.3335;x[1]=1.1624;x[2]=-2.4131;x[3]=0.8784;x[4]=-5.974;x[5]=0.025479;x[6]=0.50777;x[7]=-0.020928;x[8]=0.012594;x[9]=0.31904;	

    for (int i=0;i<8;i++)
    {
    lambdas[i]=0.1;
    deltasg[i]=0.0;
    printf("lambdas=%f\n",lambdas[i]);
    }

    printf("\n lambdas=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",lambdas[0],lambdas[1],lambdas[2],lambdas[3],lambdas[4],lambdas[5]);

    pFile=fopen("./ForkV2/sdesired1","rb");
    fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7]);
    //x[0]=8.3335;x[1]=1.1624;x[2]=-2.4131;x[3]=0.8784;x[4]=-5.974;x[5]=0.025479;x[6]=0.50777;x[7]=-0.020928;x[8]=0.012594;x[9]=0.31904;	

    for (int i=0;i<11;i++)
    {
    sdesired1[i]=temp_x[i];
    }
    
    pFile=fopen("./ForkV2/sdesired2","rb");
      fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7]);
    //x[0]=8.3335;x[1]=1.1624;x[2]=-2.4131;x[3]=0.8784;x[4]=-5.974;x[5]=0.025479;x[6]=0.50777;x[7]=-0.020928;x[8]=0.012594;x[9]=0.31904;	

    for (int i=0;i<11;i++)
    {
    sdesired2[i]=temp_x[i];
    }
    

    GETHomogeneousMatrix(tMc,"./ConstantMatrices/tMc");  //Camera to Tool frame
    GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
    GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to force sensor frame
    GETParametersCI("./ForkV2/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
    GETParametersCI("./ForkV2/Kf", Kf, Var1); //Using these parameters as misc for debug
    GETParametersCI("./ForkV2/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
    printf("\n Desired Force from file=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",DesiredForce[0],DesiredForce[1],DesiredForce[2],DesiredForce[3],DesiredForce[4],DesiredForce[5]);
    printf("\n LambdaCartfrom file=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",LambdaCart[0],LambdaCart[1],LambdaCart[2],LambdaCart[3],LambdaCart[4],LambdaCart[5]);

    cMt=tMc.inverse();
    printf("tMc=\n");printfM(tMc);
    printf("cMt=\n");printfM(cMt);
    printf("eMt=\n");printfM(eMt);
    printf("eMf=\n");printfM(eMf);
    //************************************* FORCE VARIABLES**********************************************************************
    pFile=fopen("./ForkV2/IdentifiedForceParameter","rb");
    float MovingAverageForce[6];
    float Dxf[6]; // The deviation cause by the force

    float OmegaForce[3];
    float EulerIntegrator[6];
    vpMatrix ForceBuffer(5,6);
    vpColVector x(11); // Idenitifed mass parameters of force sensor
    vpColVector	ResolvedForce(6);
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

    ofstream OutputMeasuredbMt;	OutputMeasuredbMt.open("./ForkV2/Results/bMt");
    ofstream OutputDeltaS;	OutputDeltaS.open("./ForkV2/Results/deltaS");
    ofstream OutputS;	OutputS.open("./ForkV2/Results/S");
    ofstream Outputdq;Outputdq.open("./ForkV2/Results/dq");
    ofstream Outputq;	Outputq.open("./ForkV2/Results/q");
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./ForkV2/Results/MeasuredForce");
    ofstream OutputDesiredForce;	OutputDesiredForce.open("./ForkV2/Results/DesiredForce");
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./ForkV2/Results/ResolvedForce");
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./ForkV2/Results/MovingAverageForce");

    //*****************************************************************************************************************************
    //
    //
    //								                 END OF DECLARATIONS
    //
    //
    //*****************************************************************************************************************************


    /* 
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------

    Check FRI is OK

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		*/



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
    printf("Total Time%f \n",TotalTime);

    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------	
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
    **********************************************************************************************

    START THE CONTROL LOOP

    **********************************************************************************************
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------				
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------	
    */

    
    while ((FRI->IsMachineOK()) && timecounter<TotalTime)
    {

        // First do all the operations for all Control Stages
        timecounter+=0.02;
        FRI->WaitForKRCTick();

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

        1. Get data from FRI

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			*/

        FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
        FRI->GetMeasuredCartForcesAndTorques(Measuredforce);
        FRI->GetCurrentJacobianMatrix(FriJaco);
        FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame
        FRI->GetMeasuredCartPose(MeasuredPose);
        FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt); // Converts float to matrix


        for(int i=0;i<7;i++)
        {
            JointValuesInRad[i]=MeasuredJointValuesInRad[i]; 
        }

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

        2.Extract the data from image, update the display

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */
        // Get one frame
        IplImage* original = cvQueryFrame( capture );

        if ( !original )
        {
            fprintf( stderr, "ERROR: frame is null...\n" );
            getchar();
            break;
        }

        if(!originalThr)
        {
            originalThr = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,1);
        }

        if(!displayedImage)
        {
            displayedImage = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,3);
        }

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

									        EXTRACT PRIMITIVES

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */

        /*---------------------------------------------------------------------- 

	            FILTER 1: Greyscale threshold filter

        ------------------------------------------------------------------------ */


        cvThreshold( original, originalThr, param1, 255, CV_THRESH_BINARY );
        blobs = CBlobResult( originalThr, NULL, 255 );   	 

        /*---------------------------------------------------------------------- 

	            FILTER 2: Size/Shape threshold filter (heuristic)

        ----------------------------------------------------------------------- */

        filteringblobs(blobs,param2,param3,param4); 		 
        cvMerge( originalThr, originalThr, originalThr, NULL, displayedImage );


		 /*--------------------------------------------------------------------- 
		 --------------- IMAGE PROCESSING FINISHED GET NEW POSITION OF BLOBS----
		 ----------------------------------------------------------------------- */

        // Every iteration the camera returns many blobs within threshold
        // We must (a) for a given configuration always return the 4 points as the same by convention,        
        //             however for the same configuration we must override convention if tracking is occurring
        // 
        // Therefore we can only track once the points have been found 
        // 
        // If there is no known location try to extract,Else track the last known position


        switch(ExistingFeature)
        {
            case 0: // we do not have blobs from the previous image so must redetect
                
                
               for(int i=0;i<4;i++)
                    IndexofBlob[i]=-1; // Initialisation
               for(int i=0;i<8;i++)
                        s[i]=0; // Initialisation

                forksblobs(blobs,IndexofBlob); // Extract fork dots using a priori knowledge;
                forks4blobs(blobs,IndexofBlob); // Extract fork dots using a priori knowledge apart from those already found               

                printf("\n Index before Order 1=%d,%d,%d,%d",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
               

                
               if(ValidityIndexofBlob(IndexofBlob)) // If we have found the blobs order according to a convetion
                {   
                     printf("\n Index before Order 2=%d,%d,%d,%d",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
                     OrderForkBlobs(blobs,IndexofBlob); // Rearrange the index according to a defined convention
                     
                     printf("\nPre-Location of Blob1, Xcentre[0],Ycentre[0] distance to origin = [%f,%f,%f ]\n",Xcentre[0],Ycentre[0],(dist2D(0,Xcentre[0],0,Ycentre[0])));
                     printf("\nPre-Location of Blob2, Xcentre[1],Ycentre[1] distance to origin = [%f,%f,%f ]\n",Xcentre[1],Ycentre[1], (dist2D(0,Xcentre[1],0,Ycentre[1])));
                     printf("\nPre-Location of Blob3, Xcentre[2],Ycentre[2] distance to origin = [%f,%f,%f ]\n",Xcentre[2],Ycentre[2],(dist2D(0,Xcentre[2],0,Ycentre[2])));
                     printf("\nPre-Location of Blob4, Xcentre[3],Ycentre[3] distance to origin = [%f,%f,%f ]\n",Xcentre[3],Ycentre[3],(dist2D(0,Xcentre[3],0,Ycentre[3])));
                     fits4dotstoBlob(blobs,IndexofBlob,Xcentre,Ycentre);
                     
                     printf("\n Index after Order=%d,%d,%d,%d",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
                     printf("\nLocation of Blob1, Xcentre[0],Ycentre[0] distance to origin = [%f,%f,%f ]\n",Xcentre[0],Ycentre[0],(dist2D(0,Xcentre[0],0,Ycentre[0])));
                     printf("\nLocation of Blob2, Xcentre[1],Ycentre[1] distance to origin = [%f,%f,%f ]\n",Xcentre[1],Ycentre[1], (dist2D(0,Xcentre[1],0,Ycentre[1])));
                     printf("\nLocation of Blob3, Xcentre[2],Ycentre[2] distance to origin = [%f,%f,%f ]\n",Xcentre[2],Ycentre[2],(dist2D(0,Xcentre[2],0,Ycentre[2])));
                     printf("\nLocation of Blob4, Xcentre[3],Ycentre[3] distance to origin = [%f,%f,%f ]\n",Xcentre[3],Ycentre[3],(dist2D(0,Xcentre[3],0,Ycentre[3])));
                     
                     ExistingFeature=1;
                    printf("\n \n \n \n \n \n \n \n \n"); printf("\n \n \n \n \n \n \n \n \n");
                     
                }
          
                printf("\n Number of detected blobs=%d",blobs.GetNumBlobs());
                for(int i=0;i<6;i++)
                    V_cam[i]=0.0; // Still in detection phase so set velocity to zero
                break;

        case 1: // we have the blobs from the previous image
                 
                 // Check if the blobs are still visible in image
                 
                 printf("\n Number of detected blobs=%d",blobs.GetNumBlobs());
                 if (blobs.GetNumBlobs()<4 || !ValidityIndexofBlob(IndexofBlob))
                 {
                 ExistingFeature=0;
                 printf("\n Lost the blobs, start detection\n ");

                 break;
                 }
                
                 forksblobAllstrack(blobs,Xcentre,Ycentre,IndexofBlob);
                 printf("\n Index post tracking blobs2=%d,%d,%d,%d",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
                
                 fits4dotstoBlob(blobs,IndexofBlob,Xcentre,Ycentre); // define Xcentre and Y centre as centre of blobs
                 DefineS(Xcentre,Ycentre,s);
                 printf("\n Index post tracking blobs3=%d,%d,%d,%d",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
                

                for(int i=0;i<4;i++)
                    blobs.GetBlob(IndexofBlob[i])->FillBlob( displayedImage, CV_RGB(0,255,0));              
                cvCircle(displayedImage,cvPoint(CursorU,CursorV),5,CV_RGB(255, 0, 0), 1,8,0);
                 printf("\n Location of Blob1, Xcentre[0],Ycentre[0] distance to origin = [%f,%f,%f ]\n",Xcentre[0],Ycentre[0],(dist2D(0,Xcentre[0],0,Ycentre[0])));
                 printf("\n Location of Blob2, Xcentre[1],Ycentre[1] distance to origin = [%f,%f,%f ]\n",Xcentre[1],Ycentre[1], (dist2D(0,Xcentre[1],0,Ycentre[1])));
                 printf("\n Location of Blob3, Xcentre[2],Ycentre[2] distance to origin = [%f,%f,%f ]\n",Xcentre[2],Ycentre[2],(dist2D(0,Xcentre[2],0,Ycentre[2])));
                 printf("\n Location of Blob4, Xcentre[3],Ycentre[3] distance to origin = [%f,%f,%f ]\n",Xcentre[3],Ycentre[3],(dist2D(0,Xcentre[3],0,Ycentre[3])));           
                cvCircle(displayedImage,cvPoint((int) s[0],(int) s[1]),5,CV_RGB(255, 0, 0), 1,8,0);
                cvCircle(displayedImage,cvPoint((int) s[2],(int) s[3]),5,CV_RGB(0, 0 , 255), 1,8,0);
                cvCircle(displayedImage,cvPoint((int) s[4],(int) s[5]),5,CV_RGB(255, 0, 255), 1,8,0);
                cvCircle(displayedImage,cvPoint((int) s[6],(int) s[7]),5,CV_RGB(0, 0, 0), 1,8,0);



                /*--------------------------------------------------------------------- 

	                        Compute Interaction Matrix

                ---------------------------------------------------------------------- */

                Xc1[0]=s[0];Xc1[1]=s[2];Xc2[0]=s[4];Xc2[1]=s[6];
                Yc1[0]=s[1];Yc1[1]=s[3];Yc2[0]=s[5];Yc2[1]=s[7];

                // Estimation of depth
                bMc=bMt*tMc;
                cMb=bMc.inverse();
                cPblob=cMb*bPblob;
                printf("Blob=");
                printfM(cPblob);
                for(int i=0;i<4;i++)
                {
                    DepthZ[i]=cPblob[2];
                }


                InteractionMatTwoPoints(Lpts1,Xc1,Yc1,DepthZ,CollineationParams);
                InteractionMatTwoPoints(Lpts2,Xc2,Yc2,DepthZ,CollineationParams);

	            KeepImageinView(Xc1,Yc1,FOVc1); // Generates a velocity to keep image in FOV
	            KeepImageinView(Xc2,Yc2,FOVc2); // Generates a velocity to keep image in FOV

	            InteractionMatFourPoints(L4pts,Lpts1,Lpts2);

                printf("L=\n");
	            printfM(L4pts);
                printf("J=\n");
                printfM(J);
                for(int i=0;i<8;i++)
                {
                deltas[i]=sdesired[i]-s[i];
                deltasg[i]=deltas[i]*0.1;
                }

                // Calculate the camera velocity
                V_cam=L4pts.pseudoInverse()*deltasg;// + Lpts1.pseudoInverse()*FOVc1+ Lpts2.pseudoInverse()*FOVc2;
                printf("\n Velocity  V_cam=[%8.3f %8.3f  %8.3f  %8.3f %8.3f %8.3f] \n",V_cam[0],V_cam[1],V_cam[2],V_cam[3],V_cam[4],V_cam[5]);
                printf("deltas=[%f,%f,%f,%f, %f,%f,%f,%f ]",deltas[0],deltas[1],deltas[2],deltas[3],deltas[4],deltas[5],deltas[6],deltas[7]);
                printf("deltasg=[%f,%f,%f,%f, %f,%f,%f,%f ]",deltasg[0],deltasg[1],deltasg[2],deltasg[3],deltasg[4],deltasg[5],deltasg[6],deltasg[7]);


                break;

            default:
                break;
        }

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		


        1. switch ControlStage

        case 0: Inital stage position the fork dots in the lower left of the image screen
        case 1: Apply the force 
        case 2: Keep force applied while moving along trajectory

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */


        switch(ControlStage)

        {

        case 0: 
            for(int i=0;i<8;i++)
               sdesired[i]=sdesired1[i];

            ControlStage=1; // Setup ready to change
	        
            for(int i=0;i<8;i++)
            {
                if( fabs(deltas[i])>15 || ExistingFeature==0)
                {
                    ControlStage=0; // If outside limits or lost connection don't move, stay in control stage
                }
            }
                   

            break;
        
        case 1:

           for(int i=0;i<8;i++)
               sdesired[i]=sdesired2[i];

            ControlStage=0;// Setup ready to change
	        
            for(int i=0;i<8;i++)
            {
                if( fabs(deltas[i]) > 15 || ExistingFeature==0)
                {
                    ControlStage=1;  // If outside limits say in control stage
                }
            }

            break;
        case 2: // Cannot change to case 2 at this moment

		sdesired[0]=389.5,sdesired[1]=254,sdesired[2]=411,sdesired[3]=317;
        sdesired[4]=474.0,sdesired[5]=226.0,sdesired[6]=493.5,sdesired[7]=290;


            break;
        default:
            break;
        }

       
        printf("\n Control stage=%d \n",ControlStage); 



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
        NormalToSurface(MovingAverageForce,OmegaForce);// Add an oreintation 


        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

									        VISION FORCE COMBINATION

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */
        CuttingSpeed=(float) CuttingVelocity;
        CuttingSpeed=CuttingSpeed/100; // /100 is a reducer of velocity

        printf("\nsdesired=[%f,%f,%f,%f, %f,%f,%f,%f ]",sdesired[0],sdesired[1],sdesired[2],sdesired[3],sdesired[4],sdesired[5],sdesired[6],sdesired[7]);
        printf("\ns=[%f,%f,%f,%f, %f,%f,%f,%f ]",s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7]);
		printf("\ndeltas=[%f,%f,%f,%f, %f,%f,%f,%f ]",deltas[0],deltas[1],deltas[2],deltas[3],deltas[4],deltas[5],deltas[6],deltas[7]);

        printf("\n Velocity  V_cam=[%8.3f %8.3f  %8.3f  %8.3f %8.3f %8.3f] \n",V_cam[0],V_cam[1],V_cam[2],V_cam[3],V_cam[4],V_cam[5]);
        ScrewTransformation(cMt,V_cam,V);
    
        printf("\n Velocity in Tool Space=[%8.3f %8.3f  %8.3f  %8.3f %8.3f %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
        for(int i=0;i<6;i++)
        {				 
        // adding in a variable gain
            V[i]=CuttingSpeed*V[i];  // Applying gain to Cartesian Directions instead of Joint		
        }

        printf("\n Velocity after gain=[%8.3f %8.3f  %8.3f  %8.3f %8.3f %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);

        /*
        ---------------------------------------------------------------------------------

        Applying Force Deviation

        ---------------------------------------------------------------------------------		
       
         */



        for(int i=0;i<6;i++)
        {				
	
            Dxf[i]=Kf[i]*(DesiredForce[i] + Kdeltaf *(DesiredForce[i]+MovingAverageForce[i])) ;// + as Force sensed=-Force applied
            V[i]=V[i]+Dxf[i];
            if(i>2)
            {
            V[i]=V[i]+(0.0*OmegaForce[i-3]);                
            }
        } 
            printf("\n MovingAverageForce :=[%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f] \n",MovingAverageForce[0],MovingAverageForce[1],MovingAverageForce[2],MovingAverageForce[3],MovingAverageForce[4],MovingAverageForce[5]);
            printf("\n OmegaForce :=[%8.3f %8.3f %8.3f] \n",OmegaForce[0],OmegaForce[1],OmegaForce[2]);	
            printf("\n Kdeltaf *(DForce[i]+AverageForce[i]) :=[%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f]",Kdeltaf *(DesiredForce[0]+MovingAverageForce[0]),Kdeltaf *(DesiredForce[1]+MovingAverageForce[1]),Kdeltaf *(DesiredForce[2]+MovingAverageForce[2]),Kdeltaf *(DesiredForce[3]+MovingAverageForce[3]),Kdeltaf *(DesiredForce[4]+MovingAverageForce[4]),Kdeltaf *(DesiredForce[5]+MovingAverageForce[5]));
            printf("\n Kfi[i]*EulerIntegrator :=[%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f] \n",Kfi[0]*EulerIntegrator[0],Kfi[1]*EulerIntegrator[1],Kfi[2]*EulerIntegrator[2],Kfi[3]*EulerIntegrator[3],Kfi[4]*EulerIntegrator[4],Kfi[5]*EulerIntegrator[5]);              
            printf("\n Total Devition due to Force in Tool Frame :=[%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f] \n",Dxf[0],Dxf[1],Dxf[2],Dxf[3],Dxf[4],Dxf[5]);
            printf("\n Velocity after deviation:\nV=[%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
   
            qdot=J.pseudoInverse()*V;



        /*
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

                Integrate qdot to calculate the q

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
        */

        for(int i=0;i<7;i++)
        {
            dq[i]=qdot[i];
            JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (cycletime*dq[i]);
        } 


        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

	            Saving Data for postprocessing 

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------				
        */
        // Measured Tool Position
        for(int i=0;i<4;i++)
        {
            OutputMeasuredbMt<< bMt[i][0] << ", "; 
            OutputMeasuredbMt<< bMt[i][1] << " ,"; 
            OutputMeasuredbMt<< bMt[i][2] << ", "; 
            OutputMeasuredbMt<< bMt[i][3] << " "; 	
            OutputMeasuredbMt<< endl;
        }

        //save deltaS
        for(int i=0;i<8;i++)
        {
        //				 OutputDeltaS<<deltas_r [i] << " ";  
            OutputDeltaS<< deltas[i] << " ";  
            OutputS<< s[i] << " ";  
        }
        OutputDeltaS << endl;
        OutputS<<endl;
        // save dq
        for(int i=0;i<7;i++)
        {
             Outputdq<< MeasuredJointValuesInRad[i] << ", "; 
        }
        Outputdq<< endl;

        for(int i=0;i<7;i++)
        {
             Outputq<< dq[i] << ", "; 
        }
        Outputq<< endl;
        // Measured Force raw from force sensor
        for(int i=0;i<6;i++)
        {
        OutputMeasuredForce<< Measuredforce [i] << " ";  //save Force Sensor
        }
        OutputMeasuredForce << endl;
        // Resolved Force after compensation
        for(int i=0;i<6;i++)
        {
            OutputResolvedForce<< ResolvedForce [i] << " ";  //save Force Sensor
        }
        OutputResolvedForce << endl;
        for(int i=0;i<6;i++)
        {
            OutputMovingAverageForce<< MovingAverageForce[i] << " ";  //save Force Sensor
        }
        OutputMovingAverageForce << endl;

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

        Send Data to Robot

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
        */

        FRI->SetCommandedJointPositions(JointValuesInRad);


        /* 
        ---------------------------------------------------------------------------------



        Housekeeping/End of Loop
                  _
           ______|_|_____
         /--------------\
        /  _         _   \
        | |_|  _    |_|   |
        |     | |         |
        |_____|_|_________|
        ---------------------------------------------------------------------------------		
        */
        printf("\n Cycle time=,%8.3f \n ",cycletime);
        printf("timecounter %f \n",timecounter);	
        printf("\n ---------------------------------------------- \n---------------------------------------------------------------------\n");	



     //   cvShowImage("Afterfirstthreshold", originalThr);
        cvShowImage( wndname, displayedImage );
        cvShowImage( "mywindow", original );

        if ( (cvWaitKey(50) & 255) == 27 ) break;
        // End of Loop
    }

    printf("Time Over\n");
    cvReleaseImage( &original );
   // cvReleaseImage( &originalThr );
    cvReleaseImage( &displayedImage );
    cvDestroyWindow( wndname );
    cvReleaseCapture( &capture );
    cvDestroyWindow( "mywindow" );

    if (!FRI->IsMachineOK())
    {
    printf("ERROR, machine is not ready.");


    return;
    }

    printf("Stopping the robot.\n");
    FRI->StopRobot();	

    //Close all the files
    OutputMeasuredbMt.close();	Outputdq.close();	OutputMeasuredForce.close();
    OutputResolvedForce.close();	Outputq.close();	OutputDeltaS.close();OutputMovingAverageForce.close();
    return;
}
