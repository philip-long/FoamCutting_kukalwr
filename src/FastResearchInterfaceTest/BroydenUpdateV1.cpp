#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/CuttingV15.h>

/*  ****************************************************************


Broyden update image space

**************************************************************** */


void BroydenUpdateV1(FastResearchInterface *FRI)
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
vpColVector cVc(6),cVc_visp;
vpMatrix J(6,7); // Jacobian Matrix
vpMatrix Linteraction;
vpRotationMatrix bRt,tRb,tRcam;
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv,camMo,bMtLimit,bMtinit,bMt,tMb,bMtd;




// Init filter
int TrackBars[8];

TrackBars[0]=51;TrackBars[1]=10000; // Thresholding
TrackBars[2]=3000;TrackBars[3]=37;
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
cvCreateTrackbar( "Execute Controller","Controls", &TrackBars[7], 1, NULL );
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


//==========================================================================================//





/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./BroydenUpdateV1/IdentifiedForceParameter","rb");
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
  

ofstream OutputS;     OutputS.open("./BroydenUpdateV1/Results/S",std::ios::out | std::ios::trunc );
ofstream OutputSd;     OutputSd.open("./BroydenUpdateV1/Results/Sd",std::ios::out | std::ios::trunc );
ofstream OutputErrorEstim;     OutputErrorEstim.open("./BroydenUpdateV1/Results/ErrorEstim",std::ios::out | std::ios::trunc );
ofstream OutputDS;    OutputDS.open("./BroydenUpdateV1/Results/Sdelta",std::ios::out | std::ios::trunc );
ofstream OutputCurveParameters;     OutputCurveParameters.open("./BroydenUpdateV1/Results/Curve",std::ios::out | std::ios::trunc );
ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./BroydenUpdateV1/Results/bMt",std::ios::out | std::ios::trunc  );
ofstream OutputdX;      OutputdX.open("./BroydenUpdateV1/Results/OutputdX",std::ios::out | std::ios::trunc  );
ofstream OutputCmdcVc;      OutputCmdcVc.open("./BroydenUpdateV1/Results/OutputCmdcVc",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./BroydenUpdateV1/Results/MeasuredForce",std::ios::out | std::ios::trunc );
ofstream OutputResolvedForce;	OutputResolvedForce.open("./BroydenUpdateV1/Results/ResolvedForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./BroydenUpdateV1/Results/MovingAverageForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageError; OutputMovingAverageError.open("./BroydenUpdateV1/Results/MovingAverageError",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageDx;      OutputMovingAverageDx.open("./BroydenUpdateV1/Results/OutputMovingAverageDx",std::ios::out | std::ios::trunc  );
ofstream OutputMovingAverageDs;      OutputMovingAverageDs.open("./BroydenUpdateV1/Results/OutputMovingAverageDs",std::ios::out | std::ios::trunc  );

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
vpHomogeneousMatrix bMc_old,bMc,cMb_old,cMb;

vpMatrix dXBuffer(20,6);
vpColVector MovingAverageDx(6);
vpMatrix ErrorBuffer(5,6);
vpColVector MovingAverageError(6);
vpMatrix dSBuffer(20,6);
vpColVector MovingAverageDs(6);


vpRotationMatrix bRc_old,bRcT_old,R_S,bRc,cRb;
vpTranslationVector bPc_old,bPc;
vpThetaUVector Sthetau;
vpMatrix A(6,6); // Jacobian Matrix
vpMatrix Ainit(6,6); // Jacobian Matrix
srand(time(NULL));
for (int i = 0; i < 6; i += 1)
{
    MovingAverageError[i]=100.0;
    for (int j = 0; j < 6; j += 1)
    {
    
        A[i][j]=((float) (rand()%1000)-500);
    }
}
Ainit=A;
printf("\n Initial Estimation=");printfM(A);





vpMatrix Lpts(6,6);
vpColVector sdesired(6);
vpColVector scurrent(6);
vpColVector Error(6);
vpColVector ds(6);
vpColVector dX(6);
vpColVector dX2(6);
vpColVector scurrent_old(6);
vpColVector bVc(6);

sdesired[0]=537.0;
sdesired[1]=499.0;
sdesired[2]=673.0;
sdesired[3]=609.0;
sdesired[4]=529.0;
sdesired[5]=624.0;

vpImagePoint CursorPoint[1];
float ublob1,vblob1,ublob2,vblob2,ublob3,vblob3;
int NoBlobs=0;
int Blob1=-1;
int CounterLoop=0;
int Initialisation=1;
float Gamma=0.1;
// intilise as desired
ublob1=sdesired[0];     
vblob1=sdesired[1];
ublob2=sdesired[2];     
vblob2=sdesired[3];
ublob3=sdesired[4];
vblob3=sdesired[5];

int Executing=0;
bMc_old=bMtinit;
float Xcentre[2];
float Ycentre[2];
float DepthZ[2];
DepthZ[0]=0.25;
DepthZ[1]=0.25;
vpColVector ErrorEstimate(6);




cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);


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
    
    tMb=bMt.inverse();tMb.extract(tRb);

    bMc=bMt*tMcam;   
    

    

    /*************************************************************************************************************

    Generates a Curve and a velocity
    *************************************************************************************************************/
   
    if(TrackBars[7]==0) 
    {
    Executing=0;
    }
    else
    {
    Executing=1;
    }

               
        /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VISION CONTORL 

            1. Acquire Image
            2. Extract the position of the blobs compute the image moment
            3. Create the Input velocity to the knife in the camera frame
      ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */
            



            //===========================================================================================//
            g.acquire(I);
	        vpImageConvert::convert(I,original);
	        if(!displayedImage)   displayedImage = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,3);
	        if(!displayedImage2)  displayedImage2 = cvCreateImage(cvGetSize(original), IPL_DEPTH_8U,1);
	        
            cvSet(displayedImage2, CV_RGB(0,0,0));


   
            // This will detect blobs and track them
            DetectingCuttingPoint1(original,displayedImage2,displayedImage,TrackBars,ublob1,vblob1,NoBlobs,Blob1);

            if(Blob1!=-1)
            {

                DetectingCuttingPoint2(original,displayedImage2,displayedImage,TrackBars,ublob2,vblob2,NoBlobs,Blob1);
                DetectingCuttingPoint2(original,displayedImage2,displayedImage,TrackBars,ublob3,vblob3,NoBlobs,Blob1);                
                scurrent[0]=ublob1;     
                scurrent[1]=vblob1;
                scurrent[2]=ublob2;     
                scurrent[3]=vblob2;
                scurrent[4]=ublob3;
                scurrent[5]=vblob3;
                //Executing=1;

            }
            else
            {
                printf("No Blobs");
               // Executing=0;
            }

           
            printf("\nublob1=%f,    vblob1=%f,\n",ublob1,vblob1);
            printf("ublob2=%f,  vblob2=%f \n",ublob2,vblob2);
            printf("ublob3=%f,  vblob3=%f \n",ublob3,vblob3);

            CursorPoint[0].set_ij((float) TrackBars[4],(float) TrackBars[5]); // i is in the "y" direction and j in the "X" direction	
            uk=(float) TrackBars[4];
            vk=(float) TrackBars[5];
            cvCircle(original,cvPoint(vk,uk),4,CV_RGB(0, 255,255), -1,8,0);

            cvShowImage( "Controls", displayedImage);
            cvShowImage( "Original", original);

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

		                                BROYDEN UPDATE

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */
        
        if(Executing)
        {
            printf("\n S_old: ");   printfM(scurrent_old);

            ds=scurrent-scurrent_old; // Obtain values of ds
           
            cMb=bMc.inverse();  cMb_old=bMc_old.inverse();         
           
            CartesianDiff(bMc_old,bMc,dX); // Change in Camera position in current camera frame

            printf("\n ds=");printfM(ds);printf("\n dX=");printfM(dX);

            Xcentre[0]=ublob1;             Xcentre[1]=ublob2;   Xcentre[2]=ublob3;
            Ycentre[0]=vblob1;             Ycentre[1]=vblob2;   Ycentre[3]=vblob3;
            

            fifoBufferInsert(dXBuffer,dX);
            LowPassFilter(dXBuffer,MovingAverageDx);
            fifoBufferInsert(dSBuffer,ds);
            LowPassFilter(dSBuffer,MovingAverageDs);

            InteractionMatTwoPoints(Lpts,Xcentre,Ycentre,DepthZ,Collineation);

            printf("Lpts: \n");printfM(Lpts);

            if(VectorNorm(dX,6)>0.00001)
            {
                printf("\n................................................\n");

                 printf("\n Updating the Jacobian Matrix dX=%f",VectorNorm(dX,6));
                
                 BroydenUpdater(A,MovingAverageDx,MovingAverageDs,Gamma);  // Actually kind of working                  

                 printf("\n A=");printfM(A);    printf("\n Ainit=");printfM(Ainit);

                 printf("\n................................................\n");

            }

            scurrent_old=scurrent;

            bMc_old=bMc;

            printf("\nsdesired=");printfM(sdesired);
            printf(" scurrent=");printfM(scurrent);
            
            // 1. Generate the Error
            for (int i = 0; i < scurrent.getRows(); i += 1)
            {
                Error[i]=sdesired[i]-scurrent[i];            
            }
            printf("Error=");printfM(Error);

        /* 
        ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

		                                VELOCITY CONTROL

        ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
        */
    
            if(Initialisation)
            {
                printf("\n Initialisaing \n ");

                CounterLoop++;
                printf("CounterLoop=%d",CounterLoop);

                ErrorEstimate=ds-(A*dX);
    
                printf("\n Error= ds - A*dx");printfM(ErrorEstimate);
              
                fifoBufferInsert(ErrorBuffer,ErrorEstimate);
                LowPassFilter(ErrorBuffer,MovingAverageError);

                 // Motion Generator
                for (int i = 0; i < 6; i += 1)
                {
                    cVc[i]=0;  
                }
            

                cVc[(CounterLoop % 600)/100]=sgnNumber(((CounterLoop % 600)-300))*0.05;
                printf("cVc=");                printfM(cVc);
                printf("MovingAverageError=");                printfM(MovingAverageError);
                // Error Checker
                if(VectorNorm(MovingAverageError,4)<0.05 && CounterLoop>100)
                {
                    Initialisation=0;
                }

                printf("CounterLoop=%d",CounterLoop);

            }
            else
            {
                cVc=(A.pseudoInverse()*Error);
            }


        ScrewTransformation(camMt,cVc,tV);

        }
        else // Executing=0
        {   
    
            for (int i = 0; i < 4; i += 1)
            {
                scurrent_old[i]=scurrent[i];
            }
            
            for (int i = 0; i < 6; i += 1)
            {
                tV[i]=0;  
            }
        }

        /*************************************************************************************************************

                         Send to FRI/Kuka robot

        *************************************************************************************************************/

        printf("\n tV=");printfM(tV);

        qdot=J.pseudoInverse()*tV;

        printf("\n qdot sent to robot");printfM(qdot);

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
        

        printf("here");
        for(int i=0;i<4;i++)
	    {
		    OutputS<< scurrent[i]<<" ,";
            OutputSd<< sdesired[i]<<" ,"; 
            OutputDS<< Error[i]<<" ,"; 
            OutputErrorEstim<< ErrorEstimate[i]<<" ,"; 
            OutputMovingAverageError<<MovingAverageError[i]<<" ,";
	    }
        printf("here");
        OutputMovingAverageError<< endl;
        OutputS<< endl;
        OutputSd<< endl;
        OutputDS<< endl;
        OutputErrorEstim<< endl;


        for(int i=0;i<6;i++)
        {
            OutputCmdcVc<<cVc[i] << " ";  //save cart pose
            OutputdX<<dX[i] << " ";  //save cart pose

            OutputMovingAverageDx<<MovingAverageDx[i] << " ";  //save cart pose
            OutputMovingAverageDs<<MovingAverageDs[i] << " ";  //save cart pose
        }

        OutputMovingAverageDx<< endl;
        OutputMovingAverageDs<< endl;
        OutputCmdcVc<< endl;
        OutputdX<< endl;


	    for(int i=0;i<4;i++)
	    {
		    OutputMeasuredbMt<< bMt[i][0] << " ,"; 
		    OutputMeasuredbMt<< bMt[i][1] << " ,"; 
		    OutputMeasuredbMt<< bMt[i][2] << ", "; 
		    OutputMeasuredbMt<< bMt[i][3] << " "; 
		    OutputMeasuredbMt<<endl;
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
OutputCmdcVc.close();
OutputMeasuredbMt.close();
OutputMeasuredForce.close();
OutputResolvedForce.close();
OutputMovingAverageForce.close();
OutputErrorEstim.close();

}


