//  ---------------------- Doxygen info ----------------------
//! \file UserDefinedTrajectory.cpp
//!
//! \brief
//! Read the external file and pass the trajecory in joint space to 
//! Kuka robot
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>
#include <stdio.h>
#include <stdlib.h>


//IRCCyN include, functions defined by user
#include <FunctionLibrary.h>

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

// ****************************************************************
// Function to get data from force sensor
//
void Camera2Base(FastResearchInterface *FRI)
{	
	float Measuredforce[6];
	float Measuredforce2[6];
	float MeasuredPose[12];
	vpColVector	ResolvedForce(6);
	vpColVector ResolvedForce2(6);
	vpHomogeneousMatrix cMt,tMb,bMt,bMc;

 	FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);
   //use first camera attached to computer
  int camera = 0;
  

   // Create a grabber
  bool reset = true; // Enable bus reset during construction (default)
  vp1394TwoGrabber g(reset);
  if (reset) 
  {
    // The experience shows that for some Marlin cameras (F131B and
    // F033C) a tempo of 1s is requested after a bus reset.
    vpTime::wait(500); // Wait 1000 ms
  }

 


 // Number of cameras connected on the bus
  unsigned int ncameras = 0;
  g.getNumCameras(ncameras);
  std::cout << "Number of cameras on the bus: " << ncameras << std::endl;

  // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
  g.setCamera(camera);
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
  g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);

 //Qi's Code to remove distortion for image
  vpCameraParameters camdistored;
  camdistored.initPersProjWithDistortion(542.4262727,547.0037326,344.778518,257.9310618,-0.3219875968,0.3907663559);

  // Create a ViSP gray image
  vpImage<unsigned char> I0;
  vpImage<unsigned char> I;
  g.acquire(I0); 
  vpImageTools::undistort(I0, camdistored, I);

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

  // Initialize the display with the image I. Display and image are
  // now link together.
  d->init(I);
  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);
  // Set the display window title
  vpDisplay::setTitle(I, "Camera's Image");
  // Display it on screen.
  vpDisplay::display(I);
  vpDisplay::flush(I);

  //define the tracker
  vpMbEdgeTracker tracker;

  /////object frame with respect to camera frame 
  vpHomogeneousMatrix cMo;

  // Load tracker config file (camera parameters and moving edge settings)
  tracker.loadConfigFile("./model/camera.xml");
  // Display the moving edges, see documentation for the significations of the colour
  tracker.setDisplayMovingEdges(true);
// Load the 3D model (either a vrml file or a .cao file)
  tracker.loadModel("./model/Knife.wrl");
  // initialise an instance of vpCameraParameters with the parameters from the tracker
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);

// Loop to position the cube
    while(!vpDisplay::getClick(I,false)){
    vpDisplay::display(I);
    vpDisplay::displayCharString(I, 15, 10,
		 "click after positioning the object",
		 vpColor::red);
    vpDisplay::flush(I) ;
    }

  // Initialise the tracker by clicking on the image
  // This function looks for 
  // - a ./cube/cube.init file that defines the 3d coordinates (in meter, in the object basis) of the points used for the initialisation
  tracker.initClick(I, "./model/Knife", true);
  // display the 3D model at the given pose
  tracker.display(I,cMt, cam, vpColor::red);
  //track the model
  tracker.track(I);
  tracker.getPose(cMt);
  vpDisplay::flush(I);

 // Running the tracker of the object
  while ( 1 )
  {
    //g.acquire(I);
    g.acquire(I0); 
    vpImageTools::undistort(I0, camdistored, I);

    vpDisplay::display(I);
    

     
    //Track the object.
    tracker.track(I);
    tracker.getPose(cMo);
	std::cout << "cMt" << std::endl;
	std::cout << cMt << std::endl;  
	std::cout << "................................" << std::endl;    	
    tracker.display(I, cMt, cam, vpColor::darkRed, 1);

    vpDisplay::displayFrame (I, cMt, cam, 0.05, vpColor::blue);
    vpDisplay::flush(I);
	
	FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bMt);
    bMc=bMt*cMt.inverse();
	printfM(bMc,"\n Camera in Base frame: \n"); 

  }
  

	FRI->StopRobot();	
}





















