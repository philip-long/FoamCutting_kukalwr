
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
using namespace std;
/****************************************************************
 
This control law is a replica of the External force law, except that the position data is furnished by the camera 



*/

void ExternalVisionForce2(FastResearchInterface *FRI)
{
	/* 
	---------------------------------------------------------------------------------
	
		Declare  Camera Variables  

	---------------------------------------------------------------------------------		
	*/
	// Camera Variables
	int camera = 0;  //use first camera attached to computer
    unsigned int ncameras = 0;    // Number of cameras connected on the bus
    vpImage<unsigned char> I0;         // Create a ViSP gray image
    vpImage<unsigned char> I;

    vpMbEdgeTracker TrackerTool;     //define the tracker for Tool
    vpMbEdgeTracker TrackerObject;  // define tracker for object
    vpHomogeneousMatrix cMo,oMp,cMonew; // Object frame wrt camera frame , oMp is some offset from object in object frame
    vpHomogeneousMatrix cMt; // Tool frame wrt camera frame 
    vpCameraParameters cam;         //Camera parameters

   //  vpImage<vpRGBa> Ic;
 // 	vpVideoWriter writer;

  //Set up the bit rate
  //writer.setBitRate(1000000);
  //Set up the codec to use
  //writer.setCodec(CODEC_ID_MPEG1VIDEO);

  //writer.setFileName("./data/CI/VideoCapture.mpeg");
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

	int Cut=0; // Trigger for cutting
	vpColVector pSt(6);
	vpColVector pStdesire(6);
	vpColVector deltaS(6);	//feature s
	double NormPosition;
	double NormOrientation;
	vpColVector V(6); // Kinematic Screw
	vpMatrix J(6,7); // Jacobian Matrix
	

	vpTranslationVector tTo; //Translation vector object to tool
	vpMatrix tTo_hat; tTo_hat.eye(3); // Skew symetric matrix of object to tool
	vpMatrix By,L;	L.eye(6);//interaction matrix L
	vpRotationMatrix tRo;//  Rotation matrix object  to tool
	vpRotationMatrix bRt,bRtdes; // Rotation matrix tool to base
	vpHomogeneousMatrix tMo,tMb,bMt,bMtdes,bMo,tMo_MODEL,bMc;
	vpThetaUVector tthetauo;	 
	vpMatrix LA,LB,LC; // Blocks of the interaction matrix

	vpThetaUVector bthetaut;
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
	



	  //recording the data
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




//2. initialize the FRI
	// Create a grabber
    bool reset = true; // Enable bus reset during construction (default)
    vp1394TwoGrabber g(reset);

    if (reset) 
        {
        // The experience shows that for some Marlin cameras (F131B and
        // F033C) a tempo of 1s is requested after a bus reset.
        vpTime::wait(1000); // Wait 1000 ms
        }

	    
    g.getNumCameras(ncameras);
    std::cout << "Number of cameras on the bus: " << ncameras << std::endl;

    // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
    g.setCamera(camera);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
    g.acquire(I0); 
	printf("1\n");


    //Remove distortion for image
    vpCameraParameters camdistored;
    camdistored.initPersProjWithDistortion(542.4262727,547.0037326,344.778518,257.9310618,-0.3219875968,0.3907663559);
    vpImageTools::undistort(I0, camdistored, I);
	printf("2\n");
    //define the show window
    vpDisplay *d;
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
	printf("3\n");
    // Initialize the display with the image I. Display and image are
    // now link together.
    d->init(I);
    vpDisplay::setWindowPosition(I, 400, 100); // Window location
    vpDisplay::setTitle(I, "Camera's Image");    // Display window title
    vpDisplay::display(I);    // Display it on screen.
    vpDisplay::flush(I);

  	//vpDisplay::getImage(I,Ic);
  	//writer.open(Ic);	
    
    //1. Object Tracker
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

    TrackerObject.initClick(I, "./model/rectangle", true);



    TrackerObject.getCameraParameters(cam);    // Get Camera Parameters
   
     //display Each image
    TrackerObject.display(I,cMo, cam, vpColor::red);


    //track the model/Get current TransformationMatrix
    TrackerObject.track(I);
    TrackerObject.getPose(cMo);
    vpDisplay::flush(I);



		/* 
		------------------------------Measuredforce---------------------------------------------------
		
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

////////////////////////////////////4. now the loop where do the control
	

for(int i=0;i<6;i++) pStdesire[i] = 0;  

	for (int i=0;i<6;i++)
	{
	fd[i]=0;
	}
		/* 
		---------------------------------------------------------------------------------
		
				Control Loop

		---------------------------------------------------------------------------------		
		*/



	while ((FRI->IsMachineOK()) && timecounter<TotalTime)
	{
		timecounter+=0.02;

		FRI->WaitForKRCTick();
		/* 
		---------------------------------------------------------------------------------
		
				Get data from FRI

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
	//	printf("Kuka's Estimation of force=[ %8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f ]\n",KukaForce[0],KukaForce[1],KukaForce[2],KukaForce[3],KukaForce[4],KukaForce[5]);

		/* 
		---------------------------------------------------------------------------------
		
				Extract the data from image, update the display

		---------------------------------------------------------------------------------		
		*/
		
		g.acquire(I0); 
		vpImageTools::undistort(I0, camdistored, I);
		vpDisplay::display(I);//Display it on screen

		//track the tool/object
		TrackerObject.track(I);
		TrackerObject.getPose(cMo);
		
		// display the Object Model
	    TrackerObject.display(I, cMo, cam, vpColor::darkRed, 1);
	    // display the frame
	    vpDisplay::displayFrame (I, cMo, cam, 0.05, vpColor::blue);
		vpDisplay::flush(I);                                          //flush to show image

    //vpDisplay::getImage(I,Ic);
    //writer.saveFrame(Ic);

		/* 
		---------------------------------------------------------------------------------
		
				Define the vectors s and s*

		---------------------------------------------------------------------------------		
		*/


		// Define S and deltaS
		
		tMo=bMt.inverse()*bMc*cMo;// Find the transformation matrix between tool and object

		printfM(bMc,"\n Camera in  Base Frame: \n"); 
		printfM(cMo,"\n Object in Camera Frame: \n"); 
		printfM(tMo,"\n Object in Tool Frame (feature vector s): \n");
		
		
		tMo.extract(tRo);//Rotation matrix R from HomogeneousMatrix
		tMo.extract(tTo);//extract the deplacement vector
		tthetauo.buildFrom(tRo);//theta u representation
		Two3dimensionVector2OnedemensionVector(tTo,tthetauo,pSt);
		deltaS = pSt-pStdesire;
		printfVector(deltaS,"\n Error deltaS=",6);
		NormPosition=VectorSqrt(tTo,3);
		printf("\n NormPosition =%8.3f \n",NormPosition);
		NormOrientation=VectorSqrt(tthetauo,3);
		printf(" NormOrientation =%8.3f \n",NormOrientation);
		//Compare s and s*
		

		if((VectorSqrt(deltaS,3)<0.006)&(VectorSqrt(deltaS,6)<0.05) || (Cut==1))
		{
			Cut=1;
			oMp[0][3]=Var2;
			oMp[1][3]=-Var2;
			for (int i=0;i<6;i++)
			{
				fd[i]=DesiredForce[i];
			}
			printf("\n Location reached\n --------------------------------\n CUTTING \n --------------------------------\n ");
		}




		/* 
		---------------------------------------------------------------------------------
		
				Compute the correctional velocity

		---------------------------------------------------------------------------------		
		*/

		SkewSym(tTo,tTo_hat); // computes the skew symetric matrix

		//get the current interaction matrix L
		ComputeLw(tthetauo,By); // Calculates Matrix to convert omega to uthetadot
		
		/*
				 Build the interaction matrix by 3x3 block 
					L= [A	B]
						0	C]
		*/


		LA.eye(3);
		LB=-tTo_hat;
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


	/*printf("\nL=[");
		for (int i=0;i<6;i++)
		{
		printf("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f \n",L[i][0],L[i][1],L[i][2],L[i][3],L[i][4],L[i][5]);
		}		
	printf("]");*/

		
		
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
		printf("\n Force Deviation after Force deviation Frame Space:\nDxf=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",Dxf[0],Dxf[1],Dxf[2],Dxf[3],Dxf[4],Dxf[5]);
		printf("\n Velocity after Force deviation Frame Space:\nV=[%8.3f \n %8.3f \n %8.3f \n  %8.3f \n %8.3f \n %8.3f] \n",V[0],V[1],V[2],V[3],V[4],V[5]);
	
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
		printf("\n ---------------------------------------------- \n");	




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
	    OutputMeasuredR << bthetaut[i] << " ";  //save cart pose
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

 // 	g.close();
 //	writer.close();

	return;
}





/*

	Code to save data with name defined to external file parameters, to facilate testing
	Saving data with a different suffix correspong to user input
	


	sprintf(W_or_utheta,"%f",lamda[0]); // 	Omega ==1
	sprintf(Jinv_or_JT,"%f",lamda[1]); //   Jacobian Tranpose ==1
	sprintf(RunNum,"%f",lamda[2]); // Run Number		
	char destS[50];
	char* str="./data/CI/deltaS_";
	strcpy(destS,str);
	strcat(destS,W_or_utheta);	
	strcat(destS,"_");
	strcat(destS,Jinv_or_JT);
	strcat(destS,"_");
	strcat(destS,RunNum);	


	char destX[50];
	char* strX="./data/CI/MeasuredX_";
	strcpy(destX,strX);
	strcat(destX,W_or_utheta);	
	strcat(destX,"_");
	strcat(destX,Jinv_or_JT);
	strcat(destX,"_");
	strcat(destX,RunNum);	

	char destR[50];
	char* strR="./data/CI/MeasuredR_";
	strcpy(destR,strR);
	strcat(destR,W_or_utheta);	
	strcat(destR,"_");
	strcat(destR,Jinv_or_JT);
	strcat(destR,"_");
	strcat(destR,RunNum);	
	char W_or_utheta[10];
	char Jinv_or_JT[10];
	char RunNum[10];
	OutputDeltaS.open(destS);
	OutputMeasuredX.open(destX);
	OutputMeasuredR.open(destR);

*/	
