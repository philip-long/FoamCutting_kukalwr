#include <visp/vp1394TwoGrabber.h>
#include <visp/vpMbEdgeTracker.h>


#ifndef __ViSPConfiguration__
#define __ViSPConfiguration__


void CameraConfig(vp1394TwoGrabber& g, int camera, vp1394TwoGrabber::vp1394TwoVideoModeType mode, vp1394TwoGrabber::vp1394TwoFramerateType framerate);
void TrackerConfig(vpMbEdgeTracker& tracker, char* camerafile, char* modelfile, bool setDisplayMovingEdges);

#endif
