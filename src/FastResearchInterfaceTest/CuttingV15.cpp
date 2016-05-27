#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/CuttingV15.h>

/*  ****************************************************************


Cutting in a curve IBVS, several steps required
    - 1. Moving robot to desired position using IBVS
    - 2. Moving the robot along a curve using IBVS servoing
    - 3. Reacting to force by interoducing a visual deviation
    - 4. Moving robot along a profile rathyer than simply flat 2D
 
Try to follow a curve using IBVS

**************************************************************** */


void CuttingV15(FastResearchInterface *FRI)
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
g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
vpCameraParameters camdistored;// Camera Parameters updated 5-Nov-2013
//camdistored.initPersProjWithDistortion(1267.612068,1268.174369, 320.344917, 236.6983837,-0.1277547589,0.129434146);
// Camera Parameters updated 22-Mar-2013
camdistored.initPersProjWithDistortion(1281.735355,1279.0038,321.8758512,232.8482345, -0.1143751024,0.1155316553);

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
cvCreateTrackbar( "Grayscale Threshold", "Controls", &TrackBars[0], 255, NULL );
cvCreateTrackbar( "Maximum Blob Size", "Controls", &TrackBars[1], 20000, NULL );// Maximum blob size, these are functions of the knife
cvCreateTrackbar( "Minimum Blob Size", "Controls", &TrackBars[2], 7000, NULL );// Manimum blob size
cvCreateTrackbar( "Aspect ratio", "Controls", &TrackBars[3], 100, NULL );// Aspect ratio best is 1
cvCreateTrackbar( "Cursor Move u","Controls", &TrackBars[4], 640, NULL );
cvCreateTrackbar( "Cursor Move v","Controls", &TrackBars[5], 480, NULL );
cvCreateTrackbar( "Cutting Speed","Controls", &TrackBars[6], 100, NULL );
cvCreateTrackbar( "Automatic Cut y (1)or n (0)","Controls", &TrackBars[7], 1, NULL );
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





//===========================================================================================//
//						ViSP moment variables
//===========================================================================================//
double Ap,Bp,Cp;

double Zdesired=0.0804;


vpPlane pl;
pl.setABCD(0.,0.,1.0,0.);
pl.changeFrame(camMo);
Ap=-pl.getA()/pl.getD();
Bp=-pl.getB()/pl.getD();
Cp=-pl.getC()/pl.getD();
printf("Ap=%f,Bp=%f,Cp=%f",Ap,Bp,Cp);

vpMomentObject obj(6); // Create a source moment object with 6 as maximum order
vpMomentObject dst(6); // Create a source moment object with 6 as maximum order
vpImageIo::readPGM(Idesired,"./pics/Image0");
printf("\n Image OK");
obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
dst.setType(vpMomentObject::DENSE_FULL_OBJECT); // The object is defined by a countour polygon
dst.fromImage(Idesired,100,camdistored);



vpMomentCommon mdb_cur(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),Zdesired); //Init classic features
vpFeatureMomentCommon fmdb_cur(mdb_cur);
vpMomentCommon mdb_dst(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),Zdesired); //Init classic features
vpFeatureMomentCommon fmdb_dst(mdb_dst);

mdb_cur.updateAll(dst);
//fmdb_cur.updateAll(0.,0.,10.);
fmdb_cur.updateAll(Ap,Bp,Cp);
mdb_dst.updateAll(dst);
//fmdb_dst.updateAll(0.,0.,10.);
fmdb_dst.updateAll(Ap,Bp,Cp);
// Task definition

printf("\n \n Task Definition \n \n");
vpServo task;
task.kill(); // trying some preemptive murder to avoid memory problems
task.setServo(vpServo::EYEINHAND_CAMERA);
task.setInteractionMatrixType(vpServo::DESIRED);
//task.setInteractionMatrixType(vpServo::DESIRED);
//task.setLambda(0.04);


//lambda.initStandard(6.5,0.04,40); // lambda(0) lamdba(inf) Gainslope
task.setLambda(0.1);

task.addFeature(fmdb_cur.getFeatureGravityNormalized(),fmdb_dst.getFeatureGravityNormalized());
//task.addFeature(fmdb_cur.getFeatureGravityCenter(),fmdb_dst.getFeatureGravityCenter());
task.addFeature(fmdb_cur.getFeatureAn(),fmdb_dst.getFeatureAn());
//task.addFeature(fmdb_cur.getFeatureArea(),fmdb_dst.getFeatureArea());
//task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectPx() | vpFeatureMomentCInvariant::selectPy());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC2() | vpFeatureMomentCInvariant::selectC10());
//task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectSx() | vpFeatureMomentCInvariant::selectSy());
task.addFeature(fmdb_cur.getFeatureAlpha(),fmdb_dst.getFeatureAlpha());

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


vpDisplay *ddesired;
ddesired = new vpDisplayX;
ddesired->init(Idesired);
vpDisplay::setWindowPosition(Idesired, 400, 400);// Specify the window location
vpDisplay::setTitle(Idesired, "Desired Image");// Set the display window title
vpDisplay::display(Idesired);// Display it on screen.
vpDisplay::flush(Idesired);

//==========================================================================================//





/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./CuttingV15/IdentifiedForceParameter","rb");
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
  

ofstream OutputS;     OutputS.open("./CuttingV15/Results/S",std::ios::out | std::ios::trunc );
ofstream OutputSd;     OutputSd.open("./CuttingV15/Results/Sd",std::ios::out | std::ios::trunc );
ofstream Outputuv;     Outputuv.open("./CuttingV15/Results/Uv",std::ios::out | std::ios::trunc );
ofstream OutputDS;    OutputDS.open("./CuttingV15/Results/Sdelta",std::ios::out | std::ios::trunc );
ofstream OutputCurveParameters;     OutputCurveParameters.open("./CuttingV15/Results/Curve",std::ios::out | std::ios::trunc );
ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingV15/Results/bMt",std::ios::out | std::ios::trunc  );
ofstream OutputCmdbV;      OutputCmdbV.open("./CuttingV15/Results/OutputCmdbV",std::ios::out | std::ios::trunc  );
ofstream OutputCmdtV;      OutputCmdtV.open("./CuttingV15/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingV15/Results/MeasuredForce",std::ios::out | std::ios::trunc );
ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingV15/Results/ResolvedForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingV15/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

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

// Eventually both of these will be supplied by camera
float s_areaMax=.115; 
float s_area=0.;
//float MaterialLength=-0.445;
//float MaterialHeight=.150; 
float MaterialLength=.350;

printf("cycletime=%f\n",cycletime);


/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/




// Cutting angle stuff
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Y is aligned to curve tangent => X is cutting angle
int NoBlobs=0;
int Cutting=0;

vpColVector ErrorLimit(8);
int Converged=0;
ErrorLimit[0]=0.0015;ErrorLimit[1]=0.0015;ErrorLimit[2]=0.01;
ErrorLimit[3]=0.025;ErrorLimit[4]=0.025;ErrorLimit[5]=0.005;

printf("\n \n FRI->IsMachineOK()=%d",FRI->IsMachineOK());

cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);
uk=200;vk=0;
int Forced=0; // Indicates whether passage has been affected by force deviation
int RestartCut=0; // Used to automatically restart the cutting process

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

    bMt.extract(bRt);    tMb=bMt.inverse();tMb.extract(tRb);


    /*************************************************************************************************************

    Generates a Curve and a velocity
    *************************************************************************************************************/
   
    if(TrackBars[6]>0 && (s_area>s_areaMax || bMt[1][3]>MaterialLength)) // if outside body or in reset mode  
    {
    Cutting=0;

    }
    else
    {
    Cutting=1;
    }

    
    switch (Cutting){
        case 0: // Not cutting

            printf("\n s_area %f \n\n\nRESETING \n \n \n \n Move to Safe Point bMtsafe2",s_area);
           
            // 1. Reset point Exit the media

            bMtsafe2=bMt;bMtsafe2[2][3]=bMt[2][3]+0.035;

            MovePointExp(FRI,bMtsafe2,7.0,3.0,0.01,0.1); // Move to safe point

            // 2. Move to limit + z deviation

            printf("\n Move to Safe Point bMtsafe");

            bMtsafe=bMtLimit;bMtsafe[2][3]=bMtsafe2[2][3];

            MovePointExp(FRI,bMtsafe,5.0,3.0,0.01,0.01); // Move to safe point

            // 3. Move to limit + z deviation
            printf("\n Move to bMtlimi");
            
            MovePointExp(FRI,bMtLimit,2.0,1.0,0.002,0.01); // Move to begin Passage

            // 4. Reset Variables 
            s_area=0.;Forced=0; RestartCut=0; s_desired=task.sStar;
            cvSetTrackbarPos( "Cutting Speed","Controls",0);

            break;
        case 1: 
            /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		
        
             Restart the cuting, 
            experience shows we need about 4 turns before data is updatad
            after this we can redo cutting

            */
            if(TrackBars[7])
            {
                if (RestartCut==5)
                {
                    cvSetTrackbarPos( "Cutting Speed","Controls",30);
                    RestartCut=6;
                }
                else if(RestartCut<5)RestartCut++;
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

	        DetectingCuttingPoint(original,displayedImage2,displayedImage,TrackBars,uk,vk,NoBlobs);
	        vpImageConvert::convert(displayedImage2,I2);
	        vpImageTools::binarise(I2, (unsigned char)100, (unsigned char)255, (unsigned char) 0, (unsigned char)255, (unsigned char)255);


	        if(displayd2==0)
	        {
	        d2->init(I2);
	        vpDisplay::setWindowPosition(I2, 400, 400);// Specify the window location
	        vpDisplay::setTitle(I2, "Visp Filtered Image");// Set the display window title
	        vpDisplay::display(I2);// Display it on screen.
	        vpDisplay::flush(I2);
	        displayd2=1;
	        }
//===========================================================================================
 
	        obj.fromImage(I2,100, camdistored);//update+compute moment primitives from object (for source)   
	        mdb_cur.updateAll(obj);	//update+compute features (+interaction matrixes) from plane
//	        fmdb_cur.updateAll(0.,0.,10.);
            fmdb_cur.updateAll(0.,0.,10.);
            //fmdb_cur.updateAll(Ap,Bp,Cp);
            
            task.computeControlLaw();

           // task.print();
        //    printf(" \n Lambda=%f",lambda.getLastValue());
        	err_features = task.error;             
         //   printf("infinity norm=%f",err_features.infinityNorm());
          //  printf("infinity norm=%f",err_features.euclideanNorm());
            Linteraction = task.L;//printf("\n Interaction Matrix=\n");printfM(Linteraction);

        
            if(!Forced) s_desired=task.sStar;           
        	s_current=task.s; 
        	


            vpDisplay::display(I);vpDisplay::flush(I);
            vpDisplay::display(I2);vpDisplay::flush(I2);
            vpDisplay::display(Idesired);vpDisplay::flush(Idesired);

            cvShowImage( "Controls", displayedImage);

    // Last thing to do is check everythings ok
            Converged=1;

            for (int i = 0; i < s_desired.getRows(); i += 1)
            {
                if( isnan(s_current[i]))Converged=0;   
                if(fabs(s_desired[i]-s_current[i])>ErrorLimit[i])Converged=0;              
              
            }
         
            s_area=s_current[2];
            printf("\n s_area=%f",s_area);

            if(!isfinite(s_current[2])) // check cutting conditions
            {            
                 s_area=0;
            }



            if(NoBlobs==0 && TrackBars[6]==0) // still in search mode therefore reset couter
            {
            printf("\n zero blobs");
            printf("\n \n \n Not Converged search \n \n \n");
            vk=100;
            }

            if (Converged==1) 
	        {
                uk=200;
                vk=vk+90;//(float) TrackBars[5];
                printf("\n \n \n Converged \n \n \n");
                printf("\n\n\nDetecting next point\n\n\n");
                printf("uk=%f,vk=%f",uk,vk);
	        }
            else
            {
            printf("\n Tracking, uk=%f,vk=%f \n",uk,vk);
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

            if(MovingAverageForce[1]<-0.9) // Try to filter noise
            {
            // Save the take off pose            
            //if(!Forced)bMtLimit=bMt;
            Forced=1;
            s_desired[2]-=(0.0001*MovingAverageForce[1]); // a force decreases the desired area

            }
            printf("\n scurrent=");printfM(s_current); 
            printf("\n s_desired=");printfM(s_desired); 
            /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VELOCITY CONTROL

            ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

        
            printf("bMt=\n");printfM(bMt);
          //  printf("pinv(L)=\n");printfM(Linteraction.pseudoInverse());

            vpColVector ErrorFeatures(6);
            //error*lambda=AdaptiveGainRV(linf,l0,lslope,Error,Scale) Big Scale --> smaller error
            // We can take scale as the desired value to give an idea of the scale we're working in
            ErrorFeatures[0]= AdaptiveGainRV(0.2,8.0,1.,s_desired[0]-s_current[0],0.0075);
            ErrorFeatures[1]= AdaptiveGainRV(0.2,8.0,1.,s_desired[1]-s_current[1],0.0075);
            ErrorFeatures[2]= AdaptiveGainRV(1.0,10.0,1.,s_desired[2]-s_current[2],0.0075);
            ErrorFeatures[3]= AdaptiveGainRV(0.5,0.5,1.,s_desired[3]-s_current[3],0.1);
            ErrorFeatures[4]= AdaptiveGainRV(0.09,0.09,1.,s_desired[4]-s_current[4],0.1);
            ErrorFeatures[5]= AdaptiveGainRV(0.7,10.0,1.,s_desired[5]-s_current[5],0.0075);
            printf(" \nError=       [");printfM(err_features);  
            printf("]\nError*Lambda=[");printfM(ErrorFeatures);  
            cVc=(Linteraction.pseudoInverse())*(ErrorFeatures);               


            printf("]\ncVc =");printfM(cVc);

            for (int i = 0; i < 6; i += 1){
                if( isnan(cVc[i]))cVc[i]=0;
                Vcam[i]=cVc[i];
            }         

          
         
            if ( (cvWaitKey(10) & 255) == 27 ) break;
    
            ScrewTransformation(camMt,Vcam,tV);            

          //  printf("\n Camera Velocity expressed AT & IN tool frame");printfM(tV);
            
                  
            qdot=J.pseudoInverse()*tV;
    
            printf("\n qdot sent to robot");printfM(qdot);

            /*************************************************************************************************************

            Send to FRI

            *************************************************************************************************************/

            for(int i=0;i<7;i++)
            {
                CommandedJointVelocity[i]=((float) TrackBars[6]/100  )*qdot[i];
                CommandedJointValues[i]	= MeasuredJointValues[i] + (CommandedJointVelocity[i]*cycletime);
            } 

            FRI->SetCommandedJointPositions(CommandedJointValues);


    
            break;
    }
    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    

	// Writing Data to file
         for(int i=0;i<6;i++)
	    {
		    OutputS<< s_current[i]<<" ,";
            OutputSd<< s_desired[i]<<" ,"; 
            OutputDS<< s_desired[i]-s_current[i]<<" ,"; 
	    }
        OutputS<< endl;
        OutputSd<< endl;
        OutputDS<< endl;

        for(int i=0;i<6;i++)
        {
            OutputCmdtV<< tV[i] << " ";  //save cart pose
        }
        OutputCmdtV<< endl;
        OutputCmdbV<< endl;


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

    printf("\n Forced=%d,time Counter=%f \n",Forced,timecounter);
    printf("\n-----------------------------------------------------------------------\n");
    printf("=======================================================================\n\n");
  }
	 g.close();   
    task.kill();
    printf("\n Killed \n \n");
     delete d2;	
   // cvReleaseImage( &displayedImage2 );
    // cvDestroyWindow(  "displayedImage2");
    

	FRI->StopRobot();	
	OutputS.close();
    OutputSd.close();
	OutputMeasuredbMt.close();
	OutputMeasuredForce.close();
	OutputResolvedForce.close();
	OutputMovingAverageForce.close();
    Outputuv.close();
}

