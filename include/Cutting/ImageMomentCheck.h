
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
#include <visp/vpPlane.h>

#include <visp/vpServo.h> //visual servoing task
#include<visp/vpMomentObject.h>
#include<visp/vpMomentCentered.h>
#include<visp/vpMomentCInvariant.h>
#include<visp/vpMomentCommon.h>


// Use to compute the interaction matrix
#include <visp/vpFeatureMomentGravityCenter.h>
#include <visp/vpFeatureMomentGravityCenterNormalized.h>
#include<visp/vpFeatureMomentAlpha.h>
#include<visp/vpFeatureMomentAreaNormalized.h>
#include<visp/vpFeatureMomentCInvariant.h>
#include <visp/vpFeatureMomentCommon.h> //init the feature database using the information about moment dependencies



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
#define NUMBER_OF_JOINTS		7
using namespace std;




