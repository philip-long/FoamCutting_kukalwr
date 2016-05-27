#include <visp/vpHomogeneousMatrix.h>
#include "cvblobs/BlobResult.h"

//fri include
#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>

#ifndef __ScriptLibrary__
#define __ScriptLibrary__

void MovePoint(FastResearchInterface *FRI,vpHomogeneousMatrix bMtdes,float TotalTime);
void MovePointExp(FastResearchInterface *FRI,vpHomogeneousMatrix Tmatrixdes,float Kp,float Kr);
void MovePointExp(FastResearchInterface *FRI,vpHomogeneousMatrix Tmatrixdes,float Kp,float Kr,float Perr,float Rerr);
void loadtrajectory(FastResearchInterface *FRI,vpMatrix& Trajectory,int NbrofTrajectpts,const char* filename);
#endif
