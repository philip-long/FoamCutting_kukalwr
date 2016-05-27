#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/CuttingV14.h>


/*  ****************************************************************


Cutting in a curve IBVS, several steps required
    - 1. Moving robot to desired position using IBVS
    - 2. Moving the robot along a curve using IBVS servoing
    - 3. Reacting to force by interoducing a visual deviation
    - 4. Moving robot along a profile rathyer than simply flat 2D

This file is a simple seroving of an image where a force generates an error in the image

and IT WORKS!

**************************************************************** */


void CuttingV14(FastResearchInterface *FRI)
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
camdistored.initPersProjWithDistortion(1267.612068,1268.174369, 320.344917, 236.6983837,-0.1277547589,0.129434146);

// Create a ViSP gray image
vpImage<unsigned char> I;
vpImage<unsigned char> Idesired;
IplImage* displayedImage = 0;



vpColVector cVcam(6),tVcam(6);


int TrackBars[7];

float JointSpeedGain=0.0;
// Init filter

IplImage* displayedImage2 = 0;
TrackBars[0]=100;TrackBars[1]=0; // Thresholding
TrackBars[2]=0;TrackBars[3]=0;
TrackBars[4]=400;TrackBars[5]=300;
TrackBars[6]=100;
cvNamedWindow("Controls", 0);
cvCreateTrackbar( "Binarise Thres 1", "Controls", &TrackBars[0], 255, NULL );
cvCreateTrackbar( "Unused", "Controls", &TrackBars[1], 100, NULL );// Maximum blob size, these are functions of the knife
cvCreateTrackbar( "Subarea i", "Controls", &TrackBars[2], 480, NULL );// Manimum blob size
cvCreateTrackbar( "Subarea j", "Controls", &TrackBars[3], 640, NULL );// Aspect ratio best is 1
cvCreateTrackbar( "Unused","Controls", &TrackBars[4], 480, NULL );
cvCreateTrackbar( "Unsed nj","Controls", &TrackBars[5], 640, NULL );
cvCreateTrackbar( "Cutting Speed","Controls", &TrackBars[6], 100, NULL );




//***************************VISP subsection declarations***************************

vpColVector bVt(6),V(6),tV(6),bOmegat(3),qdot(6),qinit(7),qt(7),Vcam(6);
vpMatrix J(6,7); // Jacobian Matrix
vpRotationMatrix bRt,tRb,tRcam;
vpHomogeneousMatrix bMtinit,bMtd,bMt,tMb;
vpColVector camPim(4),bPim(4);


// ViSP moment variables

vpMomentObject obj(6); // Create a source moment object with 6 as maximum order
vpMomentObject dst(6); // Create a source moment object with 6 as maximum order
vpImageIo::readPGM(Idesired,"./pics/Image0");
printf("\n Image OK");
obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
dst.setType(vpMomentObject::DENSE_FULL_OBJECT); // The object is defined by a countour polygon
dst.fromImage(Idesired,100,camdistored);

double Zdesired=0.28057;
vpMomentCommon mdb_cur(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),Zdesired); //Init classic features
vpFeatureMomentCommon fmdb_cur(mdb_cur);
vpMomentCommon mdb_dst(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),Zdesired); //Init classic features
vpFeatureMomentCommon fmdb_dst(mdb_dst);

mdb_cur.updateAll(dst);
fmdb_cur.updateAll(0.,0.,1.);
mdb_dst.updateAll(dst);
fmdb_dst.updateAll(0.,0.,1.);

// Task definition

printf("\n \n Task Definition \n \n");
vpServo task;
task.setServo(vpServo::EYEINHAND_CAMERA);
task.setInteractionMatrixType(vpServo::CURRENT);
task.setLambda(0.04);
//task.addFeature(fmdb_cur.getFeatureGravityNormalized(),fmdb_dst.getFeatureGravityNormalized());
task.addFeature(fmdb_cur.getFeatureGravityCenter(),fmdb_dst.getFeatureGravityCenter());
task.addFeature(fmdb_cur.getFeatureArea(),fmdb_dst.getFeatureArea());
//task.addFeature(fmdb_cur.getFeatureArea(),fmdb_dst.getFeatureArea());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectPx() | vpFeatureMomentCInvariant::selectPy());
task.addFeature(fmdb_cur.getFeatureAlpha(),fmdb_dst.getFeatureAlpha());

vpColVector err_features;
vpMatrix Linteraction;
vpColVector s_current;
vpColVector s_desired;
vpColVector cVc;



g.acquire(I);
vpDisplay *d2;
vpImage<unsigned char> I2(I.getHeight(), I.getWidth(),255);
vpImageTools::binarise(I2, (unsigned char)100, (unsigned char)255, (unsigned char) 255, (unsigned char)0, (unsigned char)0);
d2 = new vpDisplayX;
d2->init(I2);
vpDisplay::setWindowPosition(I2, 100, 100);// Specify the window location
vpDisplay::display(I2);
vpDisplay::flush(I2);
vpDisplay::setTitle(I2, "Processed Image");


vpDisplay *ddesired;
ddesired = new vpDisplayX;
ddesired->init(Idesired);
vpDisplay::setWindowPosition(Idesired, 100, 100);// Specify the window location
vpDisplay::display(Idesired);
vpDisplay::flush(Idesired);
vpDisplay::setTitle(Idesired, "Desired Image");



//==========================================================================================//



/*-------------------------------------------------------------------------------------------------------

INPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv;
float temp_x[11]; // A temporary variable 
GETHomogeneousMatrix(tMcam,"./ConstantMatrices/tMc");  //Camera to Tool frame
GETHomogeneousMatrix(eMf,"./ConstantMatrices/eMf");  //End effector to force sensor frame
GETHomogeneousMatrix(eMt,"./ConstantMatrices/eMt");  //End effector to tool frame

printf("\n tMcam=\n");printfM(tMcam);
camMt=tMcam.inverse();
tMcam.extract(tRcam);
printf("camMt=\n");printfM(tMcam);


/*-------------------------------------------------------------------------------------------------------

FORCE VARIABLES

-------------------------------------------------------------------------------------------------------*/

FILE * pFile;pFile=fopen("./CuttingV14/IdentifiedForceParameter","rb");
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
  

ofstream OutputS;     OutputS.open("./CuttingV14/Results/S",std::ios::out | std::ios::trunc );
ofstream OutputSd;     OutputSd.open("./CuttingV14/Results/Sd",std::ios::out | std::ios::trunc );
ofstream Outputuv;     Outputuv.open("./CuttingV14/Results/Uv",std::ios::out | std::ios::trunc );
ofstream OutputDS;    OutputDS.open("./CuttingV14/Results/Sdelta",std::ios::out | std::ios::trunc );
ofstream OutputCurveParameters;     OutputCurveParameters.open("./CuttingV14/Results/Curve",std::ios::out | std::ios::trunc );
ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./CuttingV14/Results/bMt",std::ios::out | std::ios::trunc  );
ofstream OutputCmdbV;      OutputCmdbV.open("./CuttingV14/Results/OutputCmdbV",std::ios::out | std::ios::trunc  );
ofstream OutputCmdtV;      OutputCmdtV.open("./CuttingV14/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./CuttingV14/Results/MeasuredForce",std::ios::out | std::ios::trunc );
ofstream OutputResolvedForce;	OutputResolvedForce.open("./CuttingV14/Results/ResolvedForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./CuttingV14/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

//*************************************END OF DECLARATIONS**********************************************************************










/*************************************************************************************************************

Initialising the robot

*************************************************************************************************************/

FRI->GetMeasuredJointPositions(MeasuredJointValues);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);


printf("\n bMtinit=\n");printfM(bMtinit);

// Eventually both of these will be supplied by camera
float MaterialHeight=.450; 
//float MaterialLength=-0.445;
//float MaterialHeight=.150; 
float MaterialLength=.350;
float CutDepth=.09; 
printf("cycletime=%f\n",cycletime);


/*************************************************************************************************************

Start the Control loop

*************************************************************************************************************/




// Cutting angle stuff
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Y is aligned to curve tangent => X is cutting angle
int Cutting=0;
float tcut=0.;

printf("\n \n FRI->IsMachineOK()=%d",FRI->IsMachineOK());

cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);

  while((FRI->IsMachineOK()))
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
   
    if(bMt[2][3]>MaterialHeight || bMt[0][3]>MaterialLength) // if outside body or in reset mode  
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

            // 1. End the camera acquisition

          //  cvReleaseImage( &displayedImage2 );

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
            MovePointExp(FRI,bMtinit,5,3.5,0.002,0.15); // Move to begin Passage

            break;
        case 1:
            
            printf("Cutting time=%f",tcut); 
            tcut+=cycletime;

  

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
            I2=I;

	        vpImageTools::binarise(I2,(unsigned char) TrackBars[0], (unsigned char)255, (unsigned char) 255, (unsigned char)0, (unsigned char)0);


//===========================================================================================
 
	        obj.fromImage(I2,100, camdistored);//update+compute moment primitives from object (for source)   
	        mdb_cur.updateAll(obj);	//update+compute features (+interaction matrixes) from plane
	        fmdb_cur.updateAll(0.,0.,1);
            task.computeControlLaw();
        
            //cout<<"\n";task.print();
        	err_features = task.error;printf("\n Error=");printfM(err_features);               
            Linteraction = task.L;printf("\n Interaction Matrix=\n");printfM(Linteraction);
        	s_current=task.s;printf("\n scurrent=");printfM(s_current);  
        	s_desired=task.sStar;printf("\n s_desired=");printfM(s_desired); 

            vpDisplay::display(I2);vpDisplay::flush(I2);

            vpDisplay::display(Idesired);vpDisplay::flush(Idesired);

           //  cvShowImage( "Controls", displayedImage2);

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
            if(MovingAverageForce[1]<-0.4) // Try to filter noise
            {
             s_desired[2]+=(1000*MovingAverageForce[1]); // a force decreases the desired area
            }
            /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VELOCITY CONTROL

            ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

            cVc=(Linteraction.pseudoInverse())*(s_desired-s_current);

	        printf("\n V =");printfM(cVc);
            for (int i = 0; i < 6; i += 1){
                if( isnan(cVc[i]))cVc[i]=0;
            }            

            for (int i = 0; i < 6; i += 1)
            {
                Vcam[i]=cVc[i];
            }
            
         
            if ( (cvWaitKey(10) & 255) == 27 ) break;
    
            printf("camMt=");printfM(camMt);
    
            ScrewTransformation(camMt,Vcam,tV);            

            printf("\n Camera Velocity expressed AT & IN tool frame");printfM(tV);
            
                                  
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
            OutputDS<< err_features[i]<<" ,"; 
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


    printf("\n time Counter=%f \n",timecounter);
    printf("\n-----------------------------------------------------------------------\n");
    printf("=======================================================================\n\n");
  }
	 g.close();   
    task.kill();
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





















