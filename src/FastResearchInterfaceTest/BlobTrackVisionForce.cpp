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
 
The idea of this function is to find a blob move to it , cut, then zoom out
and repeat, there fore the control law should have three steps
1. Locate -> move to
2. Cut/ Apply a force
3. Zoom out

*/
void BlobTrackVisionForce(FastResearchInterface *FRI)
	{
		// Open CV images The output and temporary images
	const char wndname[]="BlobExtraction";
	IplImage* originalThr = 0;
	IplImage* frame = 0;
	IplImage* displayedImage = 0;
	  // ViSP image declarations
	vpImage<unsigned char> I0;
	vpImage<unsigned char> I;
	vpDisplay *d;
	vpMbEdgeTracker TrackerObject;  // define tracker for object
	vpCameraParameters cam;      
	vpImagePoint TargetImagePoint[1];
	vpImagePoint CursorPoint[1];
	vpPoint o,C2,C3,C4;
	// Calculations variables

	int param1,param2,BlobIndex,CursorU,CursorV,GoodOrNay;
	double ublob,vblob,utest,vtest,xtest,ytest,Area,PlateArea,Xfound,Yfound,Zfound;
	double xb,yb,C1u,C1v,C2u,C2v,C3u,C3v,C4u,C4v; // Image Point of Object plane
	float VertexX[4]; 
	float VertexY[4]; 
	// Visp Variable declrations
	vpHomogeneousMatrix cMo; // Object frame wrt camera frame 
	vpHomogeneousMatrix cMt,tMc,bMc; // Tool frame wrt camera frame 
	vpColVector Pcb(4); // Tool frame wrt camera frame 
	vpColVector tPb(4);
	// Blob declarations
	CBlobResult blobs;
	int i;
	CBlob *currentBlob;
	CBlob BlobofInterest; // Blob used to find current information
	/* 
	---------------------------------------------------------------------------------

		Declare  All Variables  

	---------------------------------------------------------------------------------		
	*/

	double timecounter=0;
	float ToolPose[12];	
	float MeasuredPose[12];
	float JointStiffnessValues[7];
	float JointDampingValues[7]; 
	float JointValuesInRad[7];  // to store the joint value
	float dq[7];
	float MeasuredJointValuesInRad[7];
	float MeasuredJointTorqueValues[7];
	float DesiredTorqueValues[7];
	float Measuredforce[6];
	float KukaForce[6];
	vpColVector fd(6);
	vpColVector	ResolvedForce(6);
	vpColVector Dxf(6);



	//initialize Jacobian matrix, 6*7, all 0
	vpMatrix J_model(6,7);
 	float **FriJaco;  //to store the measured Jacobian from KUKA
  	FriJaco = new float* [6];
  	for(int i=0;i<6;i++)
	FriJaco[i] = new float[7];
		/*
			Declare all Control law parameters 		
		*/

	// Parameters taken from file, can change to save data with different suffixs
    double Kf[7]; //Force Gain Parameter
    double Var1;// 
	double TotalTime;
	double LambdaCart[6]; // Cartesian Stiffness
	double DesiredForce[6]; // Superimposed force
	double Var2;

	vpColVector pSt(6);
	vpColVector pStdesire(6);
	vpColVector deltaS(6);	//feature s
	double NormPosition;
	double NormOrientation;
	vpColVector V(6); // Kinematic Screw
	vpMatrix J(6,7); // Jacobian Matrix
	
	vpTranslationVector T_des; //Translation vector object to tool
	vpMatrix T_des_hat; T_des_hat.eye(3); // Skew symetric matrix of object to tool
	vpMatrix By,L;	L.eye(6);//interaction matrix L
	vpRotationMatrix R_des,bRt,bRtdes;//  Rotation matrix object  to tool
	
	vpHomogeneousMatrix tMo,tMb,bMt,bMtdes,bMo,oMc;
	vpHomogeneousMatrix tMblob,cMblob,bMblob,oMblob;
	vpHomogeneousMatrix M_des;
	vpThetaUVector tthetauo;	 
	vpMatrix LA,LB,LC; // Blocks of the interaction matrix
	vpThetaUVector thetau_des;
	vpThetaUVector bthetautdes;
	// FRI 	Variables
	float cycletime;
	int ResultValue = 0;   
/*

Extracting data from the file, FILEEXTRACT

*/
	GETHomogeneousMatrix(bMc,"./model/bMc");  //transformation matrix camera to base
	GETParametersCI("./gain/ParametersCI", LambdaCart, TotalTime);// Some Parameters to play, using as a Cartesian gain, Total Time
	GETParametersCI("./gain/ForceGains", Kf, Var1); //Using these parameters as misc for debug
	GETParametersCI("./gain/Force", DesiredForce, Var2);// Some Parameters to play, using as a Cartesian gain, Total Time
	printf("\n Desired Force from file=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",DesiredForce[0],DesiredForce[1],DesiredForce[2],DesiredForce[3],DesiredForce[4],DesiredForce[5]);
	
  	  //initialize write data
	ofstream OutputMeasuredX;
	ofstream OutputCommandedX;
	ofstream OutputMeasuredR;
	ofstream OutputCommandedR;
	ofstream OutputMeasuredForce;
	ofstream OutputEstimatedForce;
	ofstream OutputDeltaS;
	//ofstream OutputDeltaF;
	//creat the file to write in
	OutputMeasuredX.open("./data/CI/MeasuredX");
	OutputDeltaS.open("./data/CI/deltaS");
	OutputCommandedX.open("./data/CI/CommandedX");
	OutputMeasuredR.open("./data/CI/MeasuredR");
	OutputCommandedR.open("./data/CI/CommandedR");
	OutputMeasuredForce.open("./data/CI/MeasuredForce");
	OutputEstimatedForce.open("./data/CI/EstimatedForce");
	
	//OutputDeltaF.open("./data/CI/deltaF");      
	/* 
	---------------------------------------------------------------------------------
	---------------------------------------------------------------------------------		
	*/

	// Create a grabber
   	param1 = 150; // Initialise Parameters, Param one is the grayscale level
	param2 = 2000;// Param two is in are
     int camera = 0;  //use first camera attached to computer
     unsigned int ncameras = 0;    // Number of cameras connected on the bus
     bool reset = true; // Enable bus reset during construction (default)
     vp1394TwoGrabber g(reset);

     if (reset) 
        {
        // The experience shows that for some Marlin cameras (F131B and
        // F033C) a tempo of 1s is requested after a bus reset.
        vpTime::wait(500); // Wait 1000 ms
        }
    
	
	 g.getNumCameras(ncameras);
     std::cout << "Number of cameras on the bus: " << ncameras << std::endl;
     // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
     g.setCamera(camera);
     g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
     g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
     g.acquire(I0); 


     //Remove distortion for image
     vpCameraParameters camdistored;
     camdistored.initPersProjWithDistortion(542.4262727,547.0037326,344.778518,257.9310618,-0.3219875968,0.3907663559);
     vpImageTools::undistort(I0, camdistored, I);

	 
	  //define the show window
     #if defined(VISP_HAVE_X11)
     d = new vpDisplayX;
     #elif defined(VISP_HAVE_GTK)
     d = new vpDisplayGTK;
     #elif defined(VISP_HAVE_GDI)
     d = new vpDisplayGDI;
     #elif defined(VISP_HAVE_D3D9)
     d = new vpDisplayD3D;
     #elif defined(VISP_HAVE_OPENCV)
     d = new vpDisplayOpenCV;
     #endif

     // Initialize the display with the image I. Display and image are
     // now link together.
     d->init(I);
	  // Initialize the display with the image I. Display and image are

     vpDisplay::setWindowPosition(I, 400, 100); // Window location
     vpDisplay::setTitle(I, "Camera's Image");    // Display window title
     vpDisplay::display(I);    // Display it on screen.
     vpDisplay::flush(I);
	
	
    //2. Object Tracker
    TrackerObject.loadConfigFile("./model/camera.xml");   
    TrackerObject.setDisplayMovingEdges(true);    // Display Moving Edges
    TrackerObject.loadModel("./model/rectangle.wrl"); // Load the 3D model 
	
	 
	 while(!vpDisplay::getClick(I,false))
        {
         vpDisplay::display(I);
         vpDisplay::displayCharString(I, 15, 10,
         "click after positioning the object",
         vpColor::red);
         vpDisplay::flush(I) ;
        }
	
    
	//TrackerTool.initClick(I, "./model/Knife", true);
    TrackerObject.initClick(I, "./model/rectangle", true);



    TrackerObject.getCameraParameters(cam);    // Get Camera Parameters
   
     //display Each image
    TrackerObject.display(I,cMo, cam, vpColor::red);
    TrackerObject.track(I);
    TrackerObject.getPose(cMo);

    vpDisplay::flush(I);

   // After DOING THE TRACKING SETUP THE BLOB EXTRACTION
	// cvNamedWindow(wndname, 0); // This line was causing problems with tracker, I think the fact that it was before the tracker caused the issue!!
	
	
	 cvNamedWindow(wndname, CV_WINDOW_AUTOSIZE );
     cvCreateTrackbar( "Threshold 0-> Black, 255->White",wndname, &param1, 255, NULL );
	 cvCreateTrackbar( "Max Accepted BlobSize",wndname, &param2, 50000, NULL );
	 cvCreateTrackbar( "Cursor Move u",wndname, &CursorU, 1000, NULL );
	 cvCreateTrackbar( "Cursor Move v",wndname, &CursorV, 1000, NULL );


		/* 
		---------------------------------------------------------------------------------
		
				Check FRI is OK

		---------------------------------------------------------------------------------		
		*/


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


	for(int i=0;i<6;i++) pStdesire[i] = 0;  

		for (int i=0;i<6;i++)
		{
		fd[i]=0;
		}
		
		cMblob=cMo; // Initalise the blob position at the object frame
		oMblob=cMo.inverse()*cMblob;// Initialise this matrix
		
		/* 
		---------------------------------------------------------------------------------
		
				Control Loop

		---------------------------------------------------------------------------------		
		*/
	int ControlStage=1;	// Start the control strategy at zoom
	
	while ((FRI->IsMachineOK()) && timecounter<TotalTime)
  {
  
		// First do all the operations for all Control Stages
		timecounter+=0.02;
		FRI->WaitForKRCTick();
		
		/* 
		---------------------------------------------------------------------------------
		
				1. Get data from FRI

		---------------------------------------------------------------------------------		
		*/
		FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
		FRI->GetMeasuredJointTorques(MeasuredJointTorqueValues);
		FRI->GetMeasuredCartForcesAndTorques(Measuredforce);
		FRI->GetCurrentJacobianMatrix(FriJaco);
		FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the tool frame
		FRI->GetMeasuredCartPose(MeasuredPose);
		FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt); // Converts float to matrix
		ZeroForceSensor(bMt,Measuredforce,ResolvedForce);//Zeros sensor, cancels effects of tool mass,transforms to tool frame
		FRI->GetEstimatedExternalCartForcesAndTorques(KukaForce);
		printf("ResolvedForce=[ %8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f ]\n",ResolvedForce[0],ResolvedForce[1],ResolvedForce[2],ResolvedForce[3],ResolvedForce[4],ResolvedForce[5]);

		/* 
		---------------------------------------------------------------------------------
		
			    2.Extract the data from image, update the display

		---------------------------------------------------------------------------------		
		*/
	     g.acquire(I0);  // Acquiring Image at each iteration
         vpImageTools::undistort(I0, camdistored, I);
		 vpDisplay::display(I);	
		 // Convert image to open CV type and check its ok
		 vpImageConvert::convert(I,frame);
         if ( !frame ) 
		 {
             fprintf( stderr, "ERROR: frame is null...\n" );
             getchar();
             break;
         }
		// Initialise the filtering windows
		if(!originalThr)
			{
				originalThr = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U,1);
			}
		if(!displayedImage)
			{
				displayedImage = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U,3);
			}
		
		TrackerObject.track(I);
		TrackerObject.getPose(cMo);
	    TrackerObject.display(I, cMo, cam, vpColor::darkRed, 1);
	    vpDisplay::displayFrame (I, cMo, cam, 0.05, vpColor::blue);
	    vpDisplay::flush(I); //flush to show image	
		
		  
		  
		/* 
		---------------------------------------------------------------------------------
		
			   3. Define the vectors s and s*

		---------------------------------------------------------------------------------		
		*/
		
					/* 
					-----------------------
		
						3.1a Define the Blob work dectection

					-----------------------
					*/
	
		switch(ControlStage)
		{
		 case 0: // in blob detection mode
		

			 // Projecting the object top plane corners into the image
			 o.setWorldCoordinates (0.0,0.0,0.0); o.track(cMo); // Forward projection 
			 C2.setWorldCoordinates (-0.417,0.0,0.0);C2.track(cMo); // Forward projection 
			 C3.setWorldCoordinates (0.0,0.347,0.0); C3.track(cMo);	// Forward projection 
			 C4.setWorldCoordinates (-0.417,0.347,0.0);C4.track(cMo);// Forward projection 
			 vpMeterPixelConversion::convertPoint(cam, o.p[0],o.p[1] , C1u,C1v);
			 vpMeterPixelConversion::convertPoint(cam, C2.p[0],C2.p[1] , C2u,C2v); // Getting the image points of the plate
			 vpMeterPixelConversion::convertPoint(cam, C3.p[0],C3.p[1] , C3u,C3v);
			 vpMeterPixelConversion::convertPoint(cam, C4.p[0],C4.p[1] , C4u,C4v);
			 
			 VertexX[0]= C1u;	 VertexX[1]=C2u;	 VertexX[2]=C4u;	 VertexX[3]=C3u;
			 VertexY[0]= C1v;	 VertexY[1]=C2v;	 VertexY[2]=C4v;	 VertexY[3]=C3v;
			 printf("The points of the [VertexX , Vertex Y] are\n   O=[%f,%f] \n C2=[%f,%f], \n C3=[%f,%f], \n C4=[%f,%f] \n", C1u,C1v,C2u,C2v, C3u,C3v, C4u,C4v);
			
	  
			 // A small test to see if conversion from image points to camera space is working ok!
			 vpPixelMeterConversion::convertPoint(cam, C1u,C1v,xb,yb); 
			 printf(" \n Normalized Cordinates of (C1u,C1v)= xb yb= %f,%f",xb,yb);
			 printf(" \n Normalized Cordinates by Z =xb*Z, yb*Z= %f,%f",xb*cMo[2][3],yb*cMo[2][3]);
			 printf(" \n  X Y Z of object origin in camera frame %f,%f,%f \n",cMo[0][3],cMo[1][3] ,cMo[2][3] );
			 PlateArea=polygonArea(VertexX,VertexY,4);
			
			// ------------------------------------------------------------------
			// Find the Largest blob that exists in the object rectangle
			// ------------------------------------------------------------------
			
			// THRESHOLD 1. input image, Greyscale Threshold
			 cvThreshold( frame, originalThr, param1, 255, CV_THRESH_BINARY );
			 blobs = CBlobResult( originalThr, NULL, 255 ); // find blobs in image after threshold 1
			 
			// THRESHOLD 2. Excludes Blobs greater than to area of plate
			 blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 ); 
			 
			 
			 cvMerge( originalThr, originalThr, originalThr, NULL, displayedImage );
			
			 
			 BlobIndex=-1; Area=0.0;
			 ublob=0;
			 vblob=0;
			 for (i = 0; i < blobs.GetNumBlobs(); i++ )
			 {
				// Reset the two flags
				GoodOrNay=0.0;	
				// Select Current Blob
				BlobofInterest=blobs.GetBlob(i);
				// Find if blob is in Polygon, defined by VertexX, VertexY
				GoodOrNay=pnpoly(4, VertexX, VertexY, BlobofInterest.MinX(), BlobofInterest.MinY())*pnpoly(4, VertexX, VertexY, BlobofInterest.MaxX(), BlobofInterest.MaxY());// Find if min point of blob is on plate			
				// THRESHOLD 3. Excludes Blobs not lying within the plate area and choose the one of largest area
				if (GoodOrNay==1 && BlobofInterest.Area()>Area) // This should fill the blobs that lie in the rectangle, we select the biggest blob in this rectangle
					{
					// Fill in the blob as red
					currentBlob = blobs.GetBlob(i); 
					currentBlob->FillBlob( displayedImage, CV_RGB(255,0,0)); // All blobs in rectangle are red
					// Save coordinates of the centre of the blob
					ublob=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
					vblob=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);
					Area=BlobofInterest.Area();//Save Area as best so far
					BlobIndex=i;// Save index of the best so far blob
			//		printf("\n Best BlobofInterest %d at point,[%f,%f] lies in rectangle with Area %f \n",i ,ublob,vblob,Area);	
					}
				else
					{
					// Otherwise colour the blob in green,
					currentBlob = blobs.GetBlob(i);
					currentBlob->FillBlob( displayedImage, CV_RGB(0,255,0)); // All blobs detected outside rectangle are green
					}
				
			} 
			
			 // This should color the chosen blob blue i.e the best blob
			 if (BlobIndex>-1) // This means that it has found a blob fufilling criteria
				{			
				BlobofInterest=blobs.GetBlob(BlobIndex);
				printf("Best blob is %d with area of ,%f \n",BlobIndex,BlobofInterest.Area());
				currentBlob=blobs.GetBlob(BlobIndex);			
				currentBlob->FillBlob( displayedImage, CV_RGB(0,0,255)); // Best blob is Blue
				}

				
			 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^DEBUGING TOOLS^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//	
			 // Set target point as the point decided by the slidebar, then display it
			 TargetImagePoint[0].set_ij(vtest,utest); // i is in the "y" direction and j in the "X" direction	
			 vpDisplay::displayPoint (I, TargetImagePoint[0], vpColor::red);
			 CursorPoint[0].set_ij(CursorV,CursorU); // i is in the "y" direction and j in the "X" direction	
			 vpDisplay::displayPoint (I,  CursorPoint[0], vpColor::red);
			 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^DEBUGING TOOLS^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//
			 
			 
			 // ------------------------------------------------------------------
			 // Output of this section should be ublob,vblob i.e image coordinates
			 // 	of blob fufling all criteria
			 // ------------------------------------------------------------------ 
			 // Trying to calculate the X Y Z position of an image point 
			 // knowing that the image point should lie in the plane of the top of the object 
			 //

			// If a blob has not been detected in this iteration
			 
			 tMo=bMt.inverse()*bMc;
			 if (ublob==0 && vblob==0)
			{	
			printf("Going to last known position of blob with respect to the plate \n");
			
			//bMo=bMc*cMo;
			//bMo[2][3]=bMo[2][3]+0.1; // Zoom out to a position 100mm over the object frame
			//M_des=bMt.inverse()*bMo;
			tMblob=tMo*oMblob;
			M_des=tMblob;
			}
			else // If it has, find its coordinates in Cartesian space, specifcally in Camera space
			{
			 vpPixelMeterConversion::convertPoint(cam, ublob,vblob,xtest,ytest); //  extract normalised Coorindates of the test point
			 Pcb[0]=xtest; 		 Pcb[1]=ytest;		 Pcb[2]=1;		 Pcb[3]=1;
			 oMc=cMo.inverse(); // first find the inverse
			 cMblob[2][3]=-oMc[2][3]/( (oMc[2][0]*xtest) + (oMc[2][1]*ytest) +(oMc[2][2]));
			 cMblob[0][3]=xtest*cMblob[2][3];
			 cMblob[1][3]=ytest*cMblob[2][3];
			 oMblob=oMc*cMblob;
			 printf(" \n  X Y Z of blob point in camera frame %f,%f,%f",cMblob[0][3],cMblob[1][3],cMblob[2][3]);
			 printfM(oMblob,"\n Blob in Object Frame: \n");
			 tMblob=bMt.inverse()*bMc*cMblob;
			 bMblob=bMc*cMblob;
			 printfM(bMblob,"\n Blob in Base Frame: \n");
			 M_des=tMblob;
			}
			 
			break;
		 case(1):
					/* 
					-----------------------
		
						3.1b Zoom Out to a position just over the object frame

					-----------------------
					*/
			printf("Zooming Out \n");
			bMo=bMc*cMo;
			bMo[2][3]=bMo[2][3]+0.1; // Zoom out to a position 100mm over the object frame
			M_des=bMt.inverse()*bMo;
			break;
			
		 case(2):
			printf("Cutting \n");
			
			printf("Zooming Out \n");
			bMo=bMc*cMo;
			bMo[2][3]=bMo[2][3]+0.1; // Zoom out to a position 100mm over the object frame
			M_des=bMt.inverse()*bMo;
			break;
			
			
			
					/* 
					-----------------------
		
						3.1c Cutting or applying the force

					-----------------------
					*/	
			break;
		 default:
			break;
		}
		
		/* 
		-----------------------
		
		3.2 Full Definition s

		-----------------------
		*/
		
		
		
		M_des.extract(R_des);//Rotation matrix R from HomogeneousMatrix
		M_des.extract(T_des);//extract the deplacement vector
		thetau_des.buildFrom(R_des);//theta u representation
		Two3dimensionVector2OnedemensionVector(T_des,thetau_des,pSt);
		deltaS = pSt-pStdesire;
		printfVector(deltaS,"\n Error deltaS=",6);
		NormPosition=VectorSqrt(T_des,3);
		printf("\n NormPosition =%8.3f \n",NormPosition);
		NormOrientation=VectorSqrt(thetau_des,3);
		printf(" NormOrientation =%8.3f \n",NormOrientation);
		//Compare s and s* 
		
		
		if((VectorSqrt(deltaS,3)<0.01)&(VectorSqrt(deltaS,6)<0.075))
		{		
			printf("\n Location reached\n");
			if (ControlStage==0) // We have found the blob proceed to cut
			{
			ControlStage=2;
			}
			else if (ControlStage==1) // We have zoomed out sufficiently, look for new blob
			{
			ControlStage=0;
			}
			else
			{
			printf("Finished Cutting Back to Blob Hunt\n");
			ControlStage=0;
			// We have finished cutting
			}	
		} 
		
		
		/* 
		---------------------------------------------------------------------------------
		
				Compute the correctional velocity

		---------------------------------------------------------------------------------		
		*/ 
		
		SkewSym(T_des,T_des_hat); // computes the skew symetric matrix

		//get the current interaction matrix L
		ComputeLw(tthetauo,By); // Calculates Matrix to convert omega to uthetadot
		
		/*
				 Build the interaction matrix by 3x3 block 
					L= [A	B]
						0	C]
		*/
		
		
		LA.eye(3);
		LB=-T_des_hat;
		LC.eye(3);


		for (int i=0;i<3;i++)
			{			
			
			for(int j=0;j<3;j++)
				{

				L[i][j]=LA[i][j]; // Upper  left  
				L[i+3][j]=0;// Lower left
				L[i][j+3]=LB[i][j];// Upper right
				L[i+3][j+3]=LC[i][j];// Lower right

				}
 
			}

		V = (-L).pseudoInverse()*deltaS;// Applying the gain to the Cartesian velocity, which should be in the base frame

		for(int i=0;i<6;i++)
		{
		V[i]=-LambdaCart[i]*V[i]; // Applying gain to Cartesian Directions instead of Joint
		
		}
		
		printf("\n Velocity in Tool Frame Space:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
		

	  /*
		---------------------------------------------------------------------------------
		
			 Applying Force Deviation

		---------------------------------------------------------------------------------		
		*/
		for(int i=0;i<6;i++)
		 {					
			Dxf[i]=Kf[i]*(fd[i] + (Var1 *(fd[i]+ResolvedForce[i])));// + as Force sensed=-Force applied
			V[i]=V[i]+Dxf[i];
		} 
		//printf("\n Force Deviation after Force deviation Frame Space:\nDxf=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",Dxf[0],Dxf[1],Dxf[2],Dxf[3],Dxf[4],Dxf[5]);
		//printf("\n Velocity after Force deviation Frame Space:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
	
		V=J.pseudoInverse()*V;

		/*
		---------------------------------------------------------------------------------
		
				Integrate qdot to calculate the q

		---------------------------------------------------------------------------------		
		*/

		 for(int i=0;i<7;i++)
			 {
			dq[i]=V[i];
			JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (cycletime*dq[i]);
			 } 




		/* 
		---------------------------------------------------------------------------------		
		---------------------------------------------------------------------------------		
		*/





		/* 
		---------------------------------------------------------------------------------
		
				Send Data to Robot for Cartesian Control

		---------------------------------------------------------------------------------		
		*/



		printf("\n Cycle time=,%8.3f \n ",cycletime);
	

		FRI->SetCommandedJointTorques(DesiredTorqueValues);
		FRI->SetCommandedJointDamping(JointDampingValues);
 		FRI->SetCommandedJointStiffness(JointStiffnessValues);
		FRI->SetCommandedJointPositions(JointValuesInRad);
 
		printf("timecounter %f \n",timecounter);	
		printf("\n ---------------------------------------------- \n---------------------------------------------------------------------\n");	


				
		 //cvShowImage("Afterfirstthreshold", originalThr);
		 cvShowImage(wndname, displayedImage );	
		 cvShowImage( "mywindow", frame );
		 vpDisplay::flush(I); //flush to show image
			 
		 if ( (cvWaitKey(10) & 255) == 27 ) break;
		 // End of Loop
		


		/* 
		---------------------------------------------------------------------------------
		
				Saving Data for postprocessing

		---------------------------------------------------------------------------------		
		*/
		
		FRICartPose2vpHomogeneousMatrix(ToolPose,bMtdes); 
		bMtdes.extract(bRtdes);
		bthetautdes.buildFrom(bRtdes);

	    for(int i=0;i<3;i++)
	    {
	    OutputMeasuredX << MeasuredPose[i*4+3] << " ";  //save cart pose
	    }
	    OutputMeasuredX << endl;

	    for(int i=0;i<3;i++)
	    {
	    OutputCommandedX << ToolPose[i*4+3] << " ";  //save cart pose
	    }
	    OutputCommandedX << endl;

			    for(int i=0;i<3;i++)
	    {
	    OutputMeasuredR << thetau_des[i] << " ";  //save cart pose
	    }
	    OutputMeasuredR << endl;

	    for(int i=0;i<3;i++)
	    {
	    OutputCommandedR << bthetautdes[i] << " ";  //save cart pose
	    }
	    OutputCommandedR << endl;


		for(int i=0;i<6;i++)
		{
		OutputDeltaS<< deltaS[i] << " ";  //save cart pose
		}
		OutputDeltaS << endl;
		
		for(int i=0;i<6;i++)
		{
		OutputMeasuredForce<< ResolvedForce[i] << " ";  //save Force Sensor
		}
		OutputMeasuredForce << endl;
		

/////////////////////////////////////////////////////////////////////////////data saving finished



	}

	printf("Time Over\n");
	

	if (!FRI->IsMachineOK())
	{
		printf("RunTrajectorySimple(): ERROR, machine is not ready.");
		

		return;
	}

	printf("Stopping the robot.\n");
	FRI->StopRobot();	

	OutputMeasuredX.close();
	OutputCommandedX.close();
	OutputMeasuredForce.close();
	OutputDeltaS.close();
	
	return;
}
