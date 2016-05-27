#include <ScriptLibrary.h>
#include <FunctionLibrary.h>
#include <Cutting/ImageMomentCheck.h>

/*  ****************************************************************


In this program we select a image feature and segment it such that
we can calculate the invariant moments.
Then we move rotate the tool around wx and wy 
in order to chekc the stability of such moments

The servoed object defines the chosen image moments!

**************************************************************** */


void ImageMomentCheck(FastResearchInterface *FRI)
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
vpMatrix J(6,7); // Jacobian Matrix
vpRotationMatrix bRt,cRb,bRc,bRcamEQ,bRcd,bRcdT,bRdx,bRdy,tRb,tRcam,bRcamEQes,camdRc,camEQRb,Rdx,Rdy,bRcamEQesT,R_S;
float thetax,thetay;
vpHomogeneousMatrix bMtinit,bMtd,bMt,tMb;
vpColVector camPim(4),bPim(4),bVcam(6),camVcam(6);
vpThetaUVector Sthetau;
vpTranslationVector bPcamEQ,bPc,T_S;
// Init filter
int TrackBars[9];

TrackBars[0]=121;TrackBars[1]=4000; // Thresholding
TrackBars[2]=2000;TrackBars[3]=50;
TrackBars[4]=400;TrackBars[5]=250;
TrackBars[6]=0;
TrackBars[7]=45;
TrackBars[8]=45;
cvNamedWindow("Controls", 0);
cvCreateTrackbar( "Grayscale Threshold", "Controls", &TrackBars[0], 255, NULL );
cvCreateTrackbar( "Maximum Blob Size", "Controls", &TrackBars[1], 20000, NULL );// Maximum blob size, these are functions of the knife
cvCreateTrackbar( "Minimum Blob Size", "Controls", &TrackBars[2], 4000, NULL );// Manimum blob size
cvCreateTrackbar( "Aspect ratio", "Controls", &TrackBars[3], 100, NULL );// Aspect ratio best is 1
cvCreateTrackbar( "Cursor Move u","Controls", &TrackBars[4], 640, NULL );
cvCreateTrackbar( "Cursor Move v","Controls", &TrackBars[5], 480, NULL );
cvCreateTrackbar( "Cutting Speed","Controls", &TrackBars[6], 100, NULL );
cvCreateTrackbar( "Rx","Controls", &TrackBars[7], 90, NULL );
cvCreateTrackbar( "Ry","Controls", &TrackBars[8], 90, NULL );

/*-------------------------------------------------------------------------------------------------------

INPUT VARIABLES

-------------------------------------------------------------------------------------------------------*/
vpHomogeneousMatrix camMt,tMcam,eMf,eMt,bMtsafe,bMtsafe2,bMtv,bMcamEQ,camEQMb,camdMv,cMb,camMo,bMc,Tdx,Tdy;
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

double Zdesired=0.0904;


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

//task.addFeature(fmdb_cur.getFeatureGravityNormalized(),fmdb_dst.getFeatureGravityNormalized());
//task.addFeature(fmdb_cur.getFeatureGravityCenter(),fmdb_dst.getFeatureGravityCenter());
//task.addFeature(fmdb_cur.getFeatureAn(),fmdb_dst.getFeatureAn());
//task.addFeature(fmdb_cur.getFeatureArea(),fmdb_dst.getFeatureArea());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectPx() | vpFeatureMomentCInvariant::selectPy());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectSx() | vpFeatureMomentCInvariant::selectSy());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC1() | vpFeatureMomentCInvariant::selectC2());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC3() | vpFeatureMomentCInvariant::selectC4());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC5() | vpFeatureMomentCInvariant::selectC6());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC7() | vpFeatureMomentCInvariant::selectC8());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC9() | vpFeatureMomentCInvariant::selectC10());
task.addFeature(fmdb_cur.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),vpFeatureMomentCInvariant::selectC9() | vpFeatureMomentCInvariant::selectC5());
task.addFeature(fmdb_cur.getFeatureAlpha(),fmdb_dst.getFeatureAlpha());

vpColVector err_features;
vpMatrix Linteraction;
vpColVector s_current;
vpColVector s_desired;
vpColVector cVc,cVc_visp;
float uk,vk;

//===========================================================================================//
//						Camera/display Initialization
//===========================================================================================//
vpDisplay *d;
d = new vpDisplayX;
g.acquire(I);
//vpImageTools::binarise(I2, (unsigned char)100, (unsigned char)255, (unsigned char) 255, (unsigned char)0, (unsigned char)0);
//d->init(I);
//vpDisplay::setWindowPosition(I, 400, 400);// Specify the window location
//vpDisplay::setTitle(I, "Camera's Image");// Set the display window title
//vpDisplay::display(I);// Display it on screen.
//vpDisplay::flush(I);


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

FILE * pFile;pFile=fopen("./ImageMomentCheck/IdentifiedForceParameter","rb");
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
  

ofstream OutputS;     OutputS.open("./ImageMomentCheck/Results/S",std::ios::out | std::ios::trunc );
ofstream OutputSd;     OutputSd.open("./ImageMomentCheck/Results/Sd",std::ios::out | std::ios::trunc );
ofstream Outputuv;     Outputuv.open("./ImageMomentCheck/Results/Uv",std::ios::out | std::ios::trunc );
ofstream OutputDS;    OutputDS.open("./ImageMomentCheck/Results/Sdelta",std::ios::out | std::ios::trunc );
ofstream OutputCurveParameters;     OutputCurveParameters.open("./ImageMomentCheck/Results/Curve",std::ios::out | std::ios::trunc );
ofstream OutputMeasuredbMt;      OutputMeasuredbMt.open("./ImageMomentCheck/Results/bMt",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredbMc;      OutputMeasuredbMc.open("./ImageMomentCheck/Results/bMc",std::ios::out | std::ios::trunc  );
ofstream OutputRXYZ;      OutputRXYZ.open("./ImageMomentCheck/Results/OutputRXYZ",std::ios::out | std::ios::trunc  );
ofstream OutputCmdtV;      OutputCmdtV.open("./ImageMomentCheck/Results/OutputCmdtV",std::ios::out | std::ios::trunc  );
ofstream OutputMeasuredForce;	OutputMeasuredForce.open("./ImageMomentCheck/Results/MeasuredForce",std::ios::out | std::ios::trunc );
ofstream OutputResolvedForce;	OutputResolvedForce.open("./ImageMomentCheck/Results/ResolvedForce",std::ios::out | std::ios::trunc );
ofstream OutputMovingAverageForce; OutputMovingAverageForce.open("./ImageMomentCheck/Results/MovingAverageForce",std::ios::out | std::ios::trunc );

//*************************************END OF DECLARATIONS**********************************************************************










/*************************************************************************************************************

Initialising the robot

*************************************************************************************************************/

FRI->GetMeasuredJointPositions(MeasuredJointValues);
FRI->GetMeasuredCartPose(MeasuredPose);
FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMtinit);


printf("\n bMtinit=\n");printfM(bMtinit);
 bMcamEQ=bMtinit*tMcam;
printf("\n bMcamEQ=\n");printfM(bMcamEQ);
bMcamEQ.extract(bRcamEQ);bMcamEQ.extract(bPcamEQ);
camEQMb=bMcamEQ.inverse();camEQMb.extract(camEQRb);


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
int NoBlobs=0;
int Cutting=0;
float tcut=0.;
vpColVector ErrorLimit(8);
int Converged=0;
ErrorLimit[0]=0.0015;ErrorLimit[1]=0.0015;ErrorLimit[2]=0.1;
ErrorLimit[3]=0.02;ErrorLimit[4]=0.02;ErrorLimit[5]=0.02;ErrorLimit[6]=0.02;ErrorLimit[7]=0.005;

printf("\n \n FRI->IsMachineOK()=%d",FRI->IsMachineOK());

cycletime = FRI->GetFRICycleTime();printf("cycletime=%f\n",cycletime);
uk=200;vk=0;

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
            bMc=bMt*tMcam;       cMb=bMc.inverse();cMb.extract(cRb);
            bMc.extract(bPc);
            bMt.extract(bRc); 
  

           /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VISION CONTORL 

            1. Acquire Image
            2. Extract the position of the blobs compute the image moment
            3. Create the Input velocity to the knife in the camera frame
      ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */

   
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

            
            cVc_visp=task.computeControlLaw();
          

        	s_current=task.s;printf("\n scurrent=");printfM(s_current);  
        	s_desired=task.sStar;printf("\n s_desired=");printfM(s_desired); 


            vpDisplay::display(I);vpDisplay::flush(I);
            vpDisplay::display(I2);vpDisplay::flush(I2);
            vpDisplay::display(Idesired);vpDisplay::flush(Idesired);

            cvShowImage( "Controls", displayedImage);


            if(NoBlobs==0 && TrackBars[6]==0) // still in search mode therefore reset couter
            {
            printf("\n zero blobs");
            printf("\n \n \n Not Converged search \n \n \n");
            vk=100;
            }

            printf("Tracking, uk=%f,vk=%f \n",uk,vk);

         
            if ( (cvWaitKey(10) & 255) == 27 ) break;


            /* 
            ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

				                            VELOCITY CONTROL

            ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
            */
            for(int i=0;i<3;i++)
            {	
            Sthetau[i]=0.;
            }
            // If object is found rotate camera
            if(TrackBars[6]>0)
            {
            // Go to the position defined by Trackbar[7] & trackbar[8]
            thetax=(float) TrackBars[7]*PI/180-PI/4;
            thetay=(float) TrackBars[8]*PI/180-PI/4;
            TransRot(1,thetax,Rdx);
            TransRot(2,thetay,Rdy);

            bRcd=bRcamEQ*Rdx*Rdy;// Change from I* to desired pose
            // Obtain angular velocitiy
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                  bRcdT[i][j]=bRcd[j][i];
                }
            }

                R_S=bRc*bRcdT; // Calculate from current to desired 
                Sthetau.buildFrom(R_S);//theta u representation
                printf("Error orientation Camera=\n");printfM(Sthetau);
            }

            printf("Current Camera bMc=\n");printfM(bMc);
            printf("At Equilibrium Camera bMcamEQ=\n");printfM(bMcamEQ);

            // I should also servo positions or its open loop
            T_S=bPcamEQ-bPc;
            printf("Error position Camera=\n");printfM(T_S);
            for(int i=0;i<3;i++)
            {	
            bVcam[i]=2.0*T_S[i];
            bVcam[i+3]=-Sthetau[i];
            }

   
            RotateScrew(bVcam,camVcam,cRb);   // Rotate to camera frame
             printf("\n Camera Velocity expressed AT & IN camera frame");printfM(camVcam);
            ScrewTransformation(camMt,camVcam,tV);             // Change to tool

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


   
    /*************************************************************************************************************

    Save data

    *************************************************************************************************************/
    
    // Calculate output data:
        
        camdRc=camEQRb*bRc; // Difference between I* and current

        vpRxyzVector Rxyz;
        Rxyz.buildFrom(camdRc); // extract angles
        
	// Writing Data to file
         for(int i=0;i<s_current.getRows();i++)
	    {
		    OutputS<< s_current[i]<<" ,";
            OutputSd<< s_desired[i]<<" ,"; 
	    }
        OutputS<< endl;
        OutputSd<< endl;

        for(int i=0;i<3;i++)
        {
            OutputRXYZ<< Rxyz[i] << " ";  //save cart pose
        }
        OutputRXYZ<< endl;
        for(int i=0;i<3;i++)
        {
            Outputuv<< T_S[i] << " ";  //save cart pose
        }       
        for(int i=0;i<3;i++)
        {
            Outputuv<< Sthetau[i] << " ";  //save cart pose
        }
        Outputuv<< endl;
	    

        for(int i=0;i<4;i++)
	    {
		    OutputMeasuredbMc<< bMc[i][0] << " ,"; 
		    OutputMeasuredbMc<< bMc[i][1] << " ,"; 
		    OutputMeasuredbMc<< bMc[i][2] << ", "; 
		    OutputMeasuredbMc<< bMc[i][3] << " "; 
		    OutputMeasuredbMc<<endl;
	    }

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

