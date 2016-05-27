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

LineV1: Using the LineV1 program as a basis and we aim to move the robot so that the line is centred in the image and of a vertical orientation. 
Furthermore We need to add a velocity along the line direction.

In this particular case, we are not using ViSP or any force feedback



*/
void LineV1(FastResearchInterface *FRI)
{
        


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

    //*****************************************************************************************************************************

    //************************************* OPEN CV VARIABLES**************************************************************
    char wndname[] = "Blob Extraction";
    char tbarname1[] = "Grayscale Threshold";
    char tbarname2[] = "Maximum Blob Size";
    char tbarname3[] = "Minimum Blob Size";
    char tbarname4[] = "Minimum Length";
    int param1,param2,param3,param4,CursorU,CursorV,CuttingVelocity;
    // Here we draw the line for checking
    CvPoint pt1, pt2; // Two points used to draw the line


    IplImage* originalThr = 0;
    IplImage* original = 0;
    IplImage* displayedImage = 0;
    IplImage* WhiteImage=0;
    IplImage* InvertedImage=0;
    IplImage* inputtocanny=0;
    IplImage* r=0;
    IplImage* g=0;
    IplImage* b=0;
    int xint1,xint2,yint1,yint2;
    // Blobs
    float s[4];
    int IndexofBlob[2];
    float Xcentre[2];
    float Ycentre[2];
    float DepthZ[2];
    float CollineationParams[6];
    float CuttingSpeed; // Speed of Advancement
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob blobtest;
    CBlob Forkblob1,Forkblob2; // Blob used to find current information
    CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
    CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
    CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
    CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 
    CBlobGetMinX minX = CBlobGetMinX();
    CBlobGetMaxX maxX = CBlobGetMaxX();
    CBlobGetMinY minY = CBlobGetMinY();
    CBlobGetMaxY maxY = CBlobGetMaxY();
    //**********************************************************************************************************************

    //************************************* VISP VARIABLES*****************************************************************
    vpMatrix Lseg(4,6);
    vpMatrix Lseg_r(3,6);
    vpMatrix Lpts(4,6);
    vpColVector deltas(4);
    vpColVector sdesired(4);
    vpColVector sdelta_r(3);
    vpColVector V_cam(6);
    vpColVector V(6); // Kinematic Screw of tool frame
    vpColVector qdot(7);
    vpMatrix J(6,7); // Jacobian Matrix
    vpHomogeneousMatrix bMc,bMt,cMt,tMc,eMf,eMt;


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
    param4=80;// Param four is length ratio
  
    DepthZ[0]=0.5; // Estimation of depth
    DepthZ[1]=0.5;

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
    cvCreateTrackbar( tbarname2, wndname, &param2, 100000, NULL );// Maximum blob size, these are functions of the knife
    cvCreateTrackbar( tbarname3, wndname, &param3, 4000, NULL );// Minimum blob size
    cvCreateTrackbar( tbarname4, wndname, &param4, 1000, NULL );// Maximum Length
    cvCreateTrackbar( "Cursor Move u",wndname, &CursorU, 1000, NULL );
    cvCreateTrackbar( "Cursor Move v",wndname, &CursorV, 1000, NULL );
    cvCreateTrackbar( "Cutting Velocity",wndname, &CuttingVelocity, 1000, NULL );



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

    GETHomogeneousMatrix(tMc,"./ConstantMatrices/tMc");  //Camera to Tool frame
    GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
    GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to force sensor frame
    GETParametersCI("./LineV1/Kp", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
    GETParametersCI("./LineV1/Kf", Kf, Var1); //Using these parameters as misc for debug
    GETParametersCI("./LineV1/Fd", Kfi, Kdeltaf);// Some Parameters to play, using as a Cartesian gain, Total Time
    printf("\n Desired Force from file=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",DesiredForce[0],DesiredForce[1],DesiredForce[2],DesiredForce[3],DesiredForce[4],DesiredForce[5]);
    printf("\n LambdaCartfrom file=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",LambdaCart[0],LambdaCart[1],LambdaCart[2],LambdaCart[3],LambdaCart[4],LambdaCart[5]);

    cMt=tMc.inverse();

     //************************************* FORCE VARIABLES**********************************************************************
    FILE * pFile;pFile=fopen("./LineV1/IdentifiedForceParameter","rb");
    float MovingAverageForce[6];
    float Dxf[6]; // The deviation cause by the force
    float temp_x[11]; // A temporary variable 
    float OmegaForce[3];
    float EulerIntegrator[6];
    vpMatrix ForceBuffer(5,6);
    vpColVector x(11); // Idenitifed mass parameters of force sensor
    vpColVector	ResolvedForce(6);


    fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7],&temp_x[8],&temp_x[9],&temp_x[10]);
    //x[0]=8.3335;x[1]=1.1624;x[2]=-2.4131;x[3]=0.8784;x[4]=-5.974;x[5]=0.025479;x[6]=0.50777;x[7]=-0.020928;x[8]=0.012594;x[9]=0.31904;	

    for (int i=0;i<11;i++)
    {
    x[i]=temp_x[i];
    }

    printf("\n Xtemp=%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",temp_x[0],temp_x[1],temp_x[2],temp_x[3],temp_x[4],temp_x[5],temp_x[6],temp_x[7],temp_x[8],temp_x[9]);
    printf("\n X=%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9]);





    //************************************* OUTPUT VARIABLES**********************************************************************

    ofstream OutputMeasuredbMt;	OutputMeasuredbMt.open("./LineV1/Results/bMt");
    ofstream OutputDeltaS;	OutputDeltaS.open("./LineV1/Results/deltaS");
    ofstream OutputS;	OutputS.open("./LineV1/Results/S");
    ofstream Outputdq;Outputdq.open("./LineV1/Results/dq");
    ofstream Outputq;	Outputq.open("./LineV1/Results/q");
    ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./LineV1/Results/MeasuredForce");
    ofstream OutputDesiredForce;	OutputDesiredForce.open("./LineV1/Results/DesiredForce");
    ofstream OutputResolvedForce;	OutputResolvedForce.open("./LineV1/Results/ResolvedForce");
    ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./LineV1/Results/MovingAverageForce");

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
        InvertedImage = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,1);	
        WhiteImage= cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,1);
        cvSet(WhiteImage, CV_RGB(255,255,255));
        }

        if(!displayedImage)
        {
        displayedImage = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,3);
        r=cvCreateImage(cvGetSize(original),8,1);
        g=cvCreateImage(cvGetSize(original),8,1);
        b=cvCreateImage(cvGetSize(original),8,1);	
        }
        //Subtract the two images to invert the grayscale
        cvSub(WhiteImage, original, InvertedImage, NULL);
        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

								        EXTRACT PRIMITIVES

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */

        /*---------------------------------------------------------------------- 

	        FILTER 1: Greyscale threshold filter

        ------------------------------------------------------------------------ */


        cvThreshold( InvertedImage, originalThr, param1, 255, CV_THRESH_BINARY );
        blobs = CBlobResult( originalThr, NULL, 255 );   	 

        /*---------------------------------------------------------------------- 

	        FILTER 2: Size/Shape threshold filter (heuristic)

        ----------------------------------------------------------------------- */

        filteringline(blobs,param2,param3,param4); 
        cvMerge( originalThr, originalThr, originalThr, NULL, displayedImage );


        /*--------------------------------------------------------------------- 

         FILTER 3: Line filter/Last known location

        ----------------------------------------------------------------------- */

        // If there is no known location try to extract,Else track the last known position
        if(Detect)
        {
            Lineblob(blobs,IndexofBlob); // Extract the best line
        }
        else
        {
            Linetrack(blobs,Xcentre,Ycentre,IndexofBlob);//track the last known line
        }

        /*--------------------------------------------------------------------- 

		        Define Line

        ----------------------------------------------------------------------- */
        if(IndexofBlob[0]!=-1)
        {       
            blobs.GetBlob(IndexofBlob[0])->FillBlob( displayedImage, CV_RGB(0,0,255));
            blobtest=blobs.GetBlob(IndexofBlob[0]);
           
            fitLinetoBlob(blobtest,Xcentre,Ycentre);// This function fits that largest line to the blob	

            pt1.x = (int) Xcentre[0];			pt1.y = (int) Ycentre[0];
            pt2.x = (int) Xcentre[1];			pt2.y = (int) Ycentre[1];
            cvLine( displayedImage, pt1, pt2, CV_RGB(255,0,0), 3, 8 );
            Detect=0;
        }
        else// The tracker has lost the dots
        {
            Detect=1;
        }
        // Draw a circle for debugging purposes
        cvCircle(displayedImage,cvPoint(CursorU,CursorV),5,CV_RGB(255, 0, 0), 1,8,0);
        // Define s
        s[0]=(Xcentre[0]+Xcentre[1])/2;		
        s[1]=(Ycentre[0]+Ycentre[1])/2;
        s[2]=pow((Xcentre[0]-Xcentre[1]),2) + pow((pow((Ycentre[0]-Ycentre[1]),2)),0.5);
        s[3]=atan((Ycentre[0]-Ycentre[1])/(Xcentre[0]-Xcentre[1]));
        printf("xp=%f,yp=%f,L=%f,theta=%f \n",s[0],s[1],s[2],s[3]);

        CuttingSpeed=(float) CuttingVelocity;
        printf("CuttingSpeed=%f\n",CuttingSpeed);
        /*--------------------------------------------------------------------- 

		        Compute Interaction Matrix

        ---------------------------------------------------------------------- */

        // Find the interaction Matrix
        InteractionMatSegment(Lseg,Xcentre,Ycentre,DepthZ,CollineationParams);
        InteractionMatSegmentReduced(Lseg_r,Xcentre,Ycentre,DepthZ,CollineationParams);


        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

								        FORCE CONTORL 

        1. Find the net contact forces at the TCP
        2. Insert the most recent into a buffer (stack)
        3. Low pass filter the force data to limit noisy measurements
        4. Use Force to reorient the tool normal to the surface

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */

        ResolveForceatTCP( bMt, eMf, eMt,Measuredforce,ResolvedForce,x);
        fifoBufferInsert(ForceBuffer,ResolvedForce);
        LowPassFilter(ForceBuffer,MovingAverageForce);



        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

								         DEFINE THE TASK

        1. switch ControlStage

        case 0: Inital stage centre the line and make it vertical
        case 1: Apply the force and move along the line trajectory, force also orients the tool so it
        is normal to the surface


        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */



        switch(ControlStage)

        {

        case 0: 
            for (int i=0;i<6;i++)
            {
            DesiredForce[i]=0.0;
            EulerIntegrator[i]=0.0;
            }
            sdesired[0]=320,sdesired[1]=240,sdesired[2]=106,sdesired[3]=PI/2;
            sdesired[1]=s[1]; // Y is controlled during centring process
            DefineDeltaS(Xcentre,Ycentre,sdesired,deltas);
            OmegaForce[0]=0.0;OmegaForce[2]=0.0;OmegaForce[2]=0.0;

            if (fabs(deltas[3])>PI/2) // Since dots are symetric two solutions for orientation
            {		   
            printf("Changing Sdesired");
            sdesired[3]=sdesired[3]-PI;
            DefineDeltaS(Xcentre,Ycentre,sdesired,deltas);
            }

            printf("deltas=%f,%f,%f,%f \n",deltas[0],deltas[1],deltas[2],deltas[3]);
            sdelta_r[0]=deltas[0];sdelta_r[1]=deltas[1];sdelta_r[2]=deltas[3];

            // 
            if( (fabs(sdelta_r[0])<20) & ((fabs(sdelta_r[1])<10)) &  ((fabs(sdelta_r[2])<0.2)) )
            {
            
            ControlStage=1;
            printf("\n Location reached changing to controlstage=%d \n",ControlStage); 

            }

        break;



        case 1:

            sdesired[0]=320,sdesired[1]=240,sdesired[2]=106,sdesired[3]=PI/2;

            sdesired[0]=sdesired[0]+(CuttingSpeed*cos(s[3])); // Adds the cutting speed
            sdesired[1]=s[1]+(CuttingSpeed*fabs(sin(s[3]))); // Cutting speed is variable set by cutting
            DefineDeltaS(Xcentre,Ycentre,sdesired,deltas);
            DesiredForce[2]=2.0; // Adding a desired force
            NormalToSurface(MovingAverageForce,OmegaForce);// Add an oreintation 
            


            for(int i=0;i<6;i++)
            {            
            EulerIntegrator[i]=EulerIntegrator[i]+((MovingAverageForce[i]+DesiredForce[i])*cycletime);

            if(fabs(EulerIntegrator[i])>100)
            {
                EulerIntegrator[i]= EulerIntegrator[i]>0 ? 100 : -100 ;
            }

            }

            if (fabs(deltas[3])>PI/2)
            {		   
            printf("Changing Sdesired");
            sdesired[3]=sdesired[3]-PI;
            DefineDeltaS(Xcentre,Ycentre,sdesired,deltas);
            }
            // We reduced S since L is uncontrolled
            printf("deltas=%f,%f,%f,%f \n",deltas[0],deltas[1],deltas[2],deltas[3]);
            sdelta_r[0]=deltas[0];sdelta_r[1]=deltas[1];sdelta_r[2]=deltas[3];
        


            break;
        default:
        break;
        }


        printf("\n Control stage=%d \n",ControlStage); 



        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

								        VISION FORCE COMBINATION

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */

        if (!Detect) // the control law must only fire if dots have been found
        {
            V_cam=Lseg_r.pseudoInverse()*sdelta_r;
            //	V_cam=Lseg.pseudoInverse()*deltas;

            ScrewTransformation(cMt,V_cam,V);
            
            for(int i=0;i<6;i++)
            {
                V[i]=LambdaCart[i]*V[i];  // Applying gain to Cartesian Directions instead of Joint		
            }

            printf("\n Velocity in Tool Frame Space2:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);

            /*
            ---------------------------------------------------------------------------------

            Applying Force Deviation
                    1.
                    2. to oreint the tool so it is normal to the surface
            ---------------------------------------------------------------------------------		
            */
            for(int i=0;i<6;i++)
            {				
		
                Dxf[i]=Kf[i]*(DesiredForce[i] + (Kdeltaf *(DesiredForce[i]+MovingAverageForce[i])) + Kfi[i]*EulerIntegrator[i]) ;// + as Force sensed=-Force applied
                //Dxf[i]=Kf[i]*(DesiredForce[i] + (Kdeltaf *(DesiredForce[i]+MovingAverageForce[i])));
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
            for(int i=0;i<4;i++)
            {
            //				 OutputDeltaS<<sdelta_r [i] << " ";  
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

        } 
        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
        End of detection Loop

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
        */



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




        cvShowImage("Afterfirstthreshold", originalThr);
        cvShowImage( wndname, displayedImage );
        cvShowImage( "mywindow", original );

        if ( (cvWaitKey(1) & 255) == 27 ) break;
        // End of Loop
    }

    printf("Time Over\n");
    cvReleaseImage( &original );
    cvReleaseImage( &inputtocanny );
    cvReleaseImage( &originalThr );
    cvReleaseImage( &displayedImage );
    cvDestroyWindow( wndname );
    cvReleaseCapture( &capture );
    cvDestroyWindow( "mywindow" );
    fclose(pFile);
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
