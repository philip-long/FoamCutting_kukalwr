#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/CuttingV15.h>

/*  ****************************************************************


Broyden update position
 In this case the Jacobian that we learn is simply a screw transformation matrix
**************************************************************** */


void BroydenUpdateV2(FastResearchInterface *FRI)
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




/*-------------------------------------------------------------------------------------------------------

ViSP VARIABLES

-------------------------------------------------------------------------------------------------------*/

//===========================================================================================//
//						ViSP image/camera/display variables
//===========================================================================================//


// Create the variables requried for the camera
bool reset = true; // Enable bus reset during construction (default)
vp1394TwoGrabber g(reset);
if (reset) 
{
vpTime::wait(50); // Wait 1000 ms
}
g.setCamera(0);
g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_1280x960_MONO8);
g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_15);
vpCameraParameters camdistored;// Camera Parameters updated 5-Nov-2013
//camdistored.initPersProjWithDistortion(1267.612068,1268.174369, 320.344917, 236.6983837,-0.1277547589,0.129434146);
// Camera Parameters updated 22-Mar-2013
camdistored.initPersProjWithDistortion(1281.735355,1279.0038,321.8758512,232.8482345, -0.1143751024,0.1155316553);

float Collineation[5];
Collineation[0]=1281.735355;
Collineation[1]=1279.0038;
Collineation[2]=0.0;
Collineation[3]=321.8758512;
Collineation[4]=232.8482345;

// Create a ViSP gray image
vpImage<unsigned char> I;
vpImage<unsigned char> I2;
vpImage<unsigned char> Idesired;

IplImage* original=0;
IplImage* displayedImage = 0;
IplImage* displayedImage2 = 0;





//-------------------------------------------------------------------------------------------//
//						ViSP vector/matrix
//-------------------------------------------------------------------------------------------//

vpColVector cVcam(6),tVcam(6);
vpColVector bVt(6),V(6),tV(6),bOmegat(3),qdot(6),qinit(7),qt(7),Vcam(6);
vpColVector camPim(4),bPim(4);
vpColVector err_features;
vpColVector s_current;
vpColVector s_desired;
vpColVector cVc,cVc_visp;
vpMatrix J(6,7); // Jacobian Matrix
vpMatrix Linteraction;
vpRotationMatrix bRt,tRb,tRcam;
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv,camMo,bMtLimit,bMtinit,bMt,tMb,bMtd;




// Init filter
int TrackBars[8];

TrackBars[0]=121;TrackBars[1]=4000; // Thresholding
TrackBars[2]=4500;TrackBars[3]=37;
TrackBars[4]=400;TrackBars[5]=250;
TrackBars[6]=0;TrackBars[7]=0;
cvNamedWindow("Controls", 0);
cvNamedWindow("Original", 0);
cvCreateTrackbar( "Grayscale Threshold", "Controls", &TrackBars[0], 255, NULL );
cvCreateTrackbar( "Maximum Blob Size", "Controls", &TrackBars[1], 20000, NULL );// Maximum blob size, these are functions of the knife
cvCreateTrackbar( "Minimum Blob Size", "Controls", &TrackBars[2], 7000, NULL );// Manimum blob size
cvCreateTrackbar( "Aspect ratio", "Controls", &TrackBars[3], 100, NULL );// Aspect ratio best is 1
cvCreateTrackbar( "Cursor Move u","Controls", &TrackBars[4], 960, NULL );
cvCreateTrackbar( "Cursor Move v","Controls", &TrackBars[5], 1200, NULL );
cvCreateTrackbar( "Cutting Speed","Controls", &TrackBars[6], 100, NULL );
cvCreateTrackbar( "Automatic","Controls", &TrackBars[7], 1, NULL );
/*-------------------------------------------------------------------------------------------------------

INPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/


float temp_x[11]; // A temporary variable 
GETHomogeneousMatrix(tMcam,"./ConstantMatrices/tMc");  //Camera to Tool frame
GETHomogeneousMatrix(camMo,"./ConstantMatrices/cMo");  //Camera to Tool frame
GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame

printf("\n tMcam=\n");printfM(tMcam);
camMt=tMcam.inverse();
tMcam.extract(tRcam);
printf("camMt=\n");printfM(tMcam);


float uk,vk;

//===========================================================================================//
//						Camera/display Initialization
//===========================================================================================//
vpDisplay *d;
d = new vpDisplayX;
g.acquire(I);

vpDisplay *d2;
d2 = new vpDisplayX;
int displayd2=0; // a latching mechanism to intilise the image 

/*
vpDisplay *ddesired;
ddesired = new vpDisplayX;
ddesired->init(Idesired);
vpDisplay::setWindowPosition(Idesired, 400, 400);// Specify the window location
vpDisplay::setTitle(Idesired, "Desired Image");// Set the display window title
vpDisplay::display(Idesired);// Display it on screen.
vpDisplay::flush(Idesired);
*/


//==========================================================================================//





/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./BroydenUpdateV2/IdentifiedForceParameter","rb");
float MovingAverageForce[6];
vpMatrix ForceBuffer(5,6);
vpColVector x(11); // Idenitifed mass parameters of force sensor
vpColVector	ResolvedForce(6);

fscanf(pFile,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f",&temp_x[0],&temp_x[1],&temp_x[2],&temp_x[3],&temp_x[4],&temp_x[5],&temp_x[6],&temp_x[7],&temp_x[8],&temp_x[9],&temp_x[10]);
	

for (int i=0;i<11;i++)
{
x[i]=temp_x[i];
}


printf("\n X=%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[9]);


/*-------------------------------------------------------------------------------------------------------

OUTPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
  

ofstream OutputS;     OutputS.open("./BroydenUpdateV2/Results/S",std::ios::out | std::ios::trunc );
ofstream OutputSd;     OutputSd.open("./BroydenUpdateV2/Results/Sd",std::ios::out | std::ios::trunc );
ofstream OutputErrorEstim;     OutputErrorEstim.open("./BroydenUpdateV2/Results/ErrorEstim",std::ios::out | std::ios::trunc );
ofstream OutputDS;    OutputDS.open("./BroydenUpdateV2/Results/Sdelta",std::ios::out | std::ios::trunc );
ofstream OutputCurveParameters;     OutputCurveParameters.open("./BroydenUpdateV2/Results/Curve",std::ios::out | std::ios::trunc );
ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./BroydenUpdateV2/Results/bMt",std::ios::out | std::ios::trunc  );
ofstream OutputdX;      OutputdX.open("./BroydenUpdateV2/Results/OutputdX",std::ios::out | std::ios::trunc  );
ofstream OutputCmdtV;      OutputCmdtV.open("./BroydenUpdateV2/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./BroydenUpdateV2/Results/MeasuredForce",std::ios::out | std::ios::trunc );
ofstream OutputResolvedForce;	OutputResolvedForce.open("./BroydenUpdateV2/Results/ResolvedForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./BroydenUpdateV2/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

//*************************************END OF DECLARATIONS**********************************************************************










/*************************************************************************************************************

Initialising the robot

*************************************************************************************************************/

FRI->GetMeasuredJointPositions(MeasuredJointValues);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);

// bMtsafe is the safety point just before cut
// bMtLimit is last well cut point before slicing
bMtsafe=bMtinit; bMtLimit=bMtinit;

bMtsafe[2][3]=bMtinit[2][3]+0.1;

printf("\n bMtinit=\n");printfM(bMtinit);

cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);
printf("cycletime=%f\n",cycletime);


/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/

//===========================================================================================//
//						Broyden Update Variables
//===========================================================================================//
vpHomogeneousMatrix bMc_old,bMc,bMcdesired,cMb;
vpRotationMatrix bRc_old,bRcT_old,R_S,bRc,bRc_des,cRb;
vpTranslationVector bPc_old,bPc,bPc_des;


vpHomogeneousMatrix bMt_old,bMtdesired;
vpRotationMatrix bRt_old,bRtT_old;
vpTranslationVector bPt_old,bPt;


vpThetaUVector Sthetaut,Sthetauc,Sthetaudes,Sthetau;
vpMatrix A(6,6); // Jacobian Matrix
vpMatrix Ainit(6,6); // Jacobian Matrix
srand(time(NULL));
for (int i = 0; i < A.getRows(); i += 1)
{
    for (int j = 0; j < A.getCols(); j += 1)
    {
    
         A[i][j]=((float) (rand()%1000)-500);
        A[i][j]=A[i][j]/500;
    }
}
Ainit=A;
printf("\n Initial Estimation=");printfM(A);


vpMatrix Lpts(4,6);
vpColVector sdesired(6);
vpColVector scurrent(6);
vpColVector Error(6);
vpColVector ds(6);
vpColVector bVc(6);
vpColVector dX(6);
vpColVector scurrent_old(6);


vpImagePoint CursorPoint[1];
float ublob1,vblob1,ublob2,vblob2;
int NoBlobs=0;
int Blob1=-1;
float Gamma=0.1;
// intilise as desired

int Executing=0;
bMc_old=bMtinit;
float Xcentre[2];
float Ycentre[2];
float DepthZ[2];
DepthZ[0]=0.25;
DepthZ[1]=0.25;
vpColVector a1(4);

bMc=bMtinit*tMcam;
bMcdesired=bMc;
bMcdesired[0][3]=bMcdesired[0][3]+0.1;
bMcdesired[1][3]=bMcdesired[1][3]-0.05;
bMcdesired[2][3]=bMcdesired[2][3]+0.1;

bMcdesired.extract(bPc_des);    bMcdesired.extract(bRc_des); 
Sthetaudes.buildFrom(bRc_des);
printf("bMcdesired=");printfM(bMcdesired);
for (int i = 0; i < 6; i += 1)
{
     if (i<3)
     {
        sdesired[i]=bPc_des[i]; 
     }
    else  
    {
        sdesired[i]=Sthetaudes[i-3];
    }
}
printf("sdesired=");printfM(bMcdesired);


for (int i = 0; i < 20; i += 1)
{
      FRI->WaitForKRCTick();
      printf("i=%d \n",i);
}

cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);
printf("cycletime=%f\n",cycletime);




  while((FRI->IsMachineOK()) && timecounter<2000.)
  {
   
    FRI->WaitForKRCTick();

  	timecounter+=cycletime;

    /*************************************************************************************************************

    Get the data from the FRI

    *************************************************************************************************************/

    FRI->GetCurrentJacobianMatrix(FriJaco);FRIJaco2vpMatrix(FriJaco,J); 

	FRI->GetMeasuredCartPose(MeasuredPose);    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);

    FRI->GetMeasuredCartForcesAndTorques(Measuredforce);

	FRI->GetMeasuredJointPositions(MeasuredJointValues);
   
    tMb=bMt.inverse();
    tMb.extract(tRb); 
    bMt.extract(bPt);


    bMc=bMt*tMcam;
    cMb=bMc.inverse();
    cMb.extract(cRb);
    bMc.extract(bRc);  
    bMc.extract(bPc);
    Sthetauc.buildFrom(bRc);


    cvShowImage( "Controls", displayedImage);

     if ( (cvWaitKey(10) & 255) == 27 ) break;
    /*************************************************************************************************************

    Generates a Curve and a velocity
    *************************************************************************************************************/
   
    if(TrackBars[6]==0) 
    {
    Executing=0;
    }
    else
    {
    Executing=1;
    }

  
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

		                            BROYDEN UPDATE

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
    
    if(Executing)
    {

        for (int i = 0; i < 6; i += 1)
        {
         if (i<3)
         {
            scurrent[i]=bPc[i]; 
         }
        else  
        {
            scurrent[i]=Sthetauc[i-3];
           }
        }

        bMc_old.extract(bPc_old);    
        bMc_old.extract(bRc_old); 

        TranposeRotMat(bRcT_old,bRc_old);
        R_S=bRc*bRcT_old; 
        Sthetau.buildFrom(R_S);

        for (int i = 0; i < 6; i += 1)
        {
             if (i<3)
             {
                ds[i]=  bPc[i]-bPc_old[i]; 
             }
            else  
            {
                ds[i]=Sthetau[i-3];
            }
        }



        bMt_old.extract(bPt_old);    
        bMt_old.extract(bRt_old); 
        TranposeRotMat(bRtT_old,bRt_old);
        R_S=bRt*bRtT_old; 
        Sthetau.buildFrom(R_S);
        for (int i = 0; i < 6; i += 1)
        {
             if (i<3)
             {
                dX[i]=  bPt[i]-bPt_old[i]; 
             }
            else  
            {
                dX[i]=Sthetaut[i-3];
            }
        }



        printf("\n ds=");printfM(ds);
        printf("\n dX=");printfM(dX);


        if(VectorNorm(dX,6)>0.000001)
        {
            printf("\n................................................\n");

              printf("\n Updating the Jacobian Matrix dX=%f",VectorNorm(dX,6));
             BroydenUpdater(A,dX,ds,Gamma);
             printf("\n A=");printfM(A);
             printf("\n Ainit=");printfM(Ainit);
             printf("\n................................................\n");

        }
        else
        {
             printf("\n Not updating the Jacobian VectorNorm=%f",VectorNorm(ds,4));
        }

    

        bMt_old=bMt;

        bMc_old=bMt*tMcam;

    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

		                            VELOCITY CONTROL

    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */
        printf("\n sdesired=");printfM(sdesired);
        printf(" scurrent=");printfM(scurrent);
        // 1. Generate the Error
        for (int i = 0; i < 6; i += 1)
        {
        Error[i]=sdesired[i]-scurrent[i];            
        }

        printf("Error=");printfM(Error);
        printf("A.pseudoInverse()");printfM(A.pseudoInverse());
//        tV=0.5*(A.pseudoInverse()*Error);
//       bVc=0.5*(Error);
        bVt=0.5*A.pseudoInverse()*(Error);


        RotateScrew(bVt,tV,tRb);
//        RotateScrew(bVc,cVc,cRb);
       // cVc[0]=0.01;
        // Maybe a mix of in the frames?
        printf("tV=");printfM(tV);
        }
    else
    {
        for (int i = 0; i < 6; i += 1)
        {
            tV[i]=0;          
        }
    }

    for (int i = 0; i < 6; i += 1) // Speed Limits
    {
        if (tV[i]>2.0)
        {            
            tV[i]=2.0;          
        }
        else if(tV[i]<-2.0)
        {            
            tV[i]=-2.0;          
        }
    }
for (int i = 3; i < 6; i += 1)
        {
            tV[i]=0;          
        }
    printf("tV=");printfM(tV);
    qdot=J.pseudoInverse()*tV;

    printf("\n qdot sent to robot");printfM(qdot);

    /*************************************************************************************************************

    Send to FRI

    *************************************************************************************************************/

    for(int i=0;i<7;i++)
    {
        CommandedJointVelocity[i]=((float) TrackBars[6]/100  )*qdot[i];
        CommandedJointValues[i]	= MeasuredJointValues[i]+ (CommandedJointVelocity[i]*cycletime);
    } 

    printf("\n bMt=");printfM(bMt);
    FRI->SetCommandedJointPositions(CommandedJointValues);


    
        
    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    

	// Writing Data to file
         for(int i=0;i<6;i++)
	    {
		    OutputS<< scurrent[i]<<" ,";
            OutputSd<< sdesired[i]<<" ,"; 
            OutputDS<< Error[i]<<" ,"; 
	    }
        OutputS<< endl;
        OutputSd<< endl;
        OutputDS<< endl;


        for(int i=0;i<6;i++)
        {
            OutputCmdtV<< tV[i] << " ";  //save cart pose
            OutputdX<<dX[i] << " ";  //save cart pose
        }
        OutputCmdtV<< endl;
        OutputdX<< endl;


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
	 g.close();   
    printf("\n Killed \n \n");
     delete d2;	
   // cvReleaseImage( &displayedImage2 );
    // cvDestroyWindow(  "displayedImage2");
    
	cvDestroyWindow( "Controls");
	cvReleaseImage( &displayedImage );
	FRI->StopRobot();	
	OutputS.close();
    OutputSd.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputResolvedForce.close();
	OutputMovingAverageForce.close();
    OutputErrorEstim.close();
}


