#include <visp/vpHomogeneousMatrix.h>
#include "cvblobs/BlobResult.h"

#ifndef __FunctionLibrary__
#define __FunctionLibrary__

// An include file containing all the functions written in IRCCyN
// *Legacy indicates programs written no longer used due to redundancy or suspect results
// the include are still present for compliation issues

// Convert the pose sent from KRC to FRI, to a ViSP homogenous matrix
void FRICartPose2vpHomogeneousMatrix(float* Pose,vpHomogeneousMatrix& M);
// Convert the Jacobian matrix sent from KRC to FRI, to a ViSP Jacobian matrix
void FRIJaco2vpMatrix(float** fjaco,vpMatrix& M);

// Three functions to append vectors together, creating a task vector
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpRzyxVector b, vpColVector& c);
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpRzyzVector b, vpColVector& c);
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpThetaUVector b, vpColVector& c);

// Change the Jacobian Frame *Legacy
void JacobianFrameChange(vpMatrix iJ, vpMatrix& sJ, vpRotationMatrix sRi);
// Compute control law *Legacy
void ComputeControlLaw(vpColVector& qdot, vpMatrix Jaco_base, vpMatrix L, double* lamda, vpColVector deltaS);
//Print a Homogenous matrix
void printfM(vpHomogeneousMatrix M,char* s);
void printfM(vpMatrix M,char* s);
// Print a Jacobian matrix
void printfJ(vpMatrix J,char* s);
void printfM(vpMatrix M); 
void printfM(vpColVector v);


// Get homogenous transormation matrix from a file
void GETHomogeneousMatrix(vpHomogeneousMatrix& M,const char* filename);
// Compute control law *Legacy
void ComputeControlLawSecondTask(vpColVector& qdot, vpMatrix Jaco_base, vpMatrix L, double* lamda, vpColVector deltaS, vpColVector z);
// Compute control law *Legacy
void JointLimitAvoid(float* q, vpColVector& z);
// Find the norm of a vector
double VectorSqrt(vpColVector v, int num);
// Find the largest abs value of a vector of n compomnents
double findabsmax(vpColVector v,int n);
//Compute control law *Legacy
void Selectqdot(vpColVector& after, vpColVector cal,vpColVector before, double t);
//Compute control law *Legacy
void ComputeControlLaw(vpColVector& V, vpMatrix L, double* lamda, vpColVector deltaS);
//Get parameters from a file
void GETParametersCI(const char* filename, double* lamda, double& intertime);
void GETParametersJ(char* filename, double* lamda, double& intertime);
// Calculate the skew symmetric matrix from a vector u
void SkewSym(vpColVector u, vpMatrix& L) ;
// Compute a matrix that converts (u theta )dot to omega
void ComputeLw(vpColVector thetau, vpMatrix& Lw) ;


// List of functions imported from MyFunctions.h /home/VisualServoingPrograms/Functions
// Fully commented there ;)

int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);
double polygonArea(float *X, float *Y, int points) ;
 double dist2D(double x1, double x2, double y1, double y2);
void filteringblobs(CBlobResult& blobs,int MaxArea, int MinArea, int AspectRatio);
void filteringline(CBlobResult& blobs,int MaxArea, int MinArea, int length);
void forksblobs(CBlobResult blobs,int* IndexofBlob);
void Lineblob(CBlobResult blobs,int* IndexofBlob);
void forksblobstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob);
void Linetrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob);
void InteractionMatSegment(vpMatrix& Lseg,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters);
void InteractionMatSegmentReduced(vpMatrix& Lseg_r,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters);
void InteractionMatTwoPoints(vpMatrix& Lpts,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters);
void InteractionMatOnePoint(vpMatrix& Lpts,float u1,float v1,float DepthZ,float* CameraParameters);
 void DefineDeltaS(float* Xcentre,float* Ycentre,vpColVector sdesired,vpColVector& deltas);
void ScrewTransformation(vpHomogeneousMatrix Tij,vpColVector Vii,vpColVector& Vjj);
void fitLinetoBlob(CBlob blobtest,float* Xcentre,float* Ycentre);
void fitsdotstoBlob(CBlobResult blobs,int* IndexofBlob,float* Xcentre,float* Ycentre);
void forks4blobs(CBlobResult blobs,int* IndexofBlob);
void forks4blobstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob);
 void fitsdotstoBlob(CBlobResult blobs,int* IndexofBlob,float* Xcentre,float* Ycentre);
void InteractionMatFourPoints(vpMatrix& L4pts,vpMatrix Lpts1,vpMatrix Lpts2);
void DefineS(float* Xcentre,float* Ycentre,vpColVector& s);
void fits4dotstoBlob(CBlobResult blobs,int* IndexofBlob,float* Xcentre,float* Ycentre);
void TrajecSects(vpColVector qinit,vpColVector& qt,float t);
void LinearTrajq(vpColVector qinit,vpColVector qfinal,vpColVector& qt,float time,float tfinal);
void OrderForkBlobs(CBlobResult blobs,int* IndexofBlob);
void forksblobAllstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int*  IndexofBlob);
void fitLinetoBlob(CBlob blobtest,float* Xcentre,float* Ycentre);
int ValidityIndexofBlob(int*  IndexofBlob);
void XTrajec(vpHomogeneousMatrix bMt_init,vpHomogeneousMatrix& bMt,float t,vpColVector& bVt,vpColVector& bOmegat);
void RotateScrew(vpColVector bV, vpColVector& wV, vpRotationMatrix wRb);
void XTrajecSim(vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtfinal,vpHomogeneousMatrix& bMt,float time,float tf,vpColVector& bVt,vpColVector& bOmegat);
float polyval(float* p,int n,float Xt);
void polyder(float* p,int n,float* y);
void CurveTraject(float* p,int n,float time,float tf,vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtFinal,vpHomogeneousMatrix& bMttraj,vpColVector& bVt);
void TransMat(float* y,vpHomogeneousMatrix& T);
void TransRot(int choice,float y,vpRotationMatrix& R);
void TransMat(int choice,float y,vpHomogeneousMatrix& T);
void CurveVelocity(vpHomogeneousMatrix bMt,float CutDepth,vpColVector CurveParameters,vpColVector& bVt);
void CurveVelocity(vpHomogeneousMatrix bMt,float CutDepth,vpColVector CurveParameters,vpHomogeneousMatrix& bMtd);
void CurveGen(float* p,int n,float time,float tf,vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtFinal,vpColVector& CurveParameters);
void CurveEval(float* p,int n,vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix& bMtd);
void CurveEval(float* p,int n,float X,vpHomogeneousMatrix& bMtd);
void ImagePointFind(IplImage* in,IplImage* Out,int *TrackBars,vpColVector& ublob,vpColVector& vblob,int& n); 
void SortImageVector(float uk,float vk,float& ukd,float& vkd,vpColVector& ublob,vpColVector& vblob,int& n);
void SquarePointFind(IplImage* in,IplImage* Out,IplImage* Out2,int *TrackBars,float uk,float vk,vpColVector& ublob,vpColVector& vblob,int& n);
void visionFramefrom3Pts(float xn1,float yn1,float xn2,float yn2,float xn3,float yn3,vpHomogeneousMatrix tMcam,vpHomogeneousMatrix bMt,vpHomogeneousMatrix& bMtv,float DepthZ);
void visionFramefrom6Pts(float xn1,float yn1,float xn2,float yn2,float xn3,float yn3,float xn4,float yn4,float xn5,float yn5,float xn6,float yn6,vpHomogeneousMatrix tMcam,vpHomogeneousMatrix bMt,vpHomogeneousMatrix& bMtv,float DepthZ);
void TranposeRotMat(vpRotationMatrix& Rt,vpRotationMatrix R);
void DetectingCuttingPoint(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs);
double AdaptiveGain(double kinf,double k0,double km, double err,double xS);
double AdaptiveGainRV(double kinf,double k0,double km, double err,double xS);
void DetectingCuttingPoint2(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs,int Blob1);
void DetectingCuttingPoint1(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs,int& Blob1);
void BroydenUpdater(vpMatrix& A,vpColVector& dX,vpColVector& ds,float Gamma);
void CartesianError(vpHomogeneousMatrix& Td,vpHomogeneousMatrix& T,vpColVector& E);
void CartesianDiff(vpHomogeneousMatrix& bMtold,vpHomogeneousMatrix& bMt,vpColVector& E);
int sgnNumber(float x);
int sgnNumber(int x);



//void loadtrajectory(vpMatrix& Trajectory,int NbrofTrajectpts,const char* filename);



// Function used to zero the force sensor
void ZeroForceSensor(vpHomogeneousMatrix bMt,float* MeasuredForce,vpColVector& ResolvedForce);
void ZeroForceSensorTest(vpHomogeneousMatrix bMt,float* MeasuredForce2,vpColVector& ResolvedForce2);
// Determine cutting trajec *Legacy
void CuttingTrajectory(double t,double tfinal,vpHomogeneousMatrix& oMp_t,vpColVector& CylinderCap1,vpColVector& CylinderCap2);
// Determine if a frame lies in cylinder
double InCylinder(vpHomogeneousMatrix oMt,vpColVector CylinderCap1,vpColVector CylinderCap2,double Guideradius);
// Function used to calibrate force sensor
void ResolveForceatTCP(vpHomogeneousMatrix bMt,vpHomogeneousMatrix eMf,vpHomogeneousMatrix eMt,float* Measuredforce2,vpColVector& ResolvedForce2,vpColVector x);
// Create a stack for the force measurements
void fifoBufferInsert(vpMatrix& ForceBuffer,vpColVector ResolvedForce);

// Low Pass Filter of the force
void LowPassFilter(vpMatrix ForceBuffer,float* MovingAverageForce);
void LowPassFilter(vpMatrix ForceBuffer,vpColVector& MovingAverageForce);
void NormalToSurface(float* MovingAverageForce,float* OmegaForce);
void KeepImageinView(float* Xcentre,float* Ycentre,vpColVector& deltas_FOV);

// Compute sinc of theta
double computesinc(double theta);

// Compute the interaction matrix *Legacy
void LPositionFixCam(vpHomogeneousMatrix bmo, vpMatrix& c);
// Compute the interaction matrix *Legacy
void LPoseFixCam(vpHomogeneousMatrix bmg,vpHomogeneousMatrix bmo, vpMatrix& c);
// Find eigenvalue of matrix
int EigenValueNum(vpMatrix J);
// Compute the Control law *Legacy
void ComputeControlLawCartesianImpedance(vpColVector& v, vpMatrix L, double* lamda, vpColVector deltaS);
// Compute the x velocity *Legacy
void Selectxdot(vpColVector& after, vpColVector cal, vpColVector before, double t, double* data);
vpHomogeneousMatrix Filter( vpHomogeneousMatrix cMoa,  vpHomogeneousMatrix cMob,  vpHomogeneousMatrix cMobb);
// Move the point of jacobian caculation *Legacy
void JacobianMovePoint(vpMatrix Jg, vpMatrix& Jt, vpHomogeneousMatrix bMg, vpHomogeneousMatrix bMt);
// Get cutting points from a file *Legacy
void GETCuttingPoints(char* filename, int num, double**& data);
int GETFileLines(char* filename);
// Force Control law *Legacy
void ComputeCommandedForce(vpColVector measured, vpColVector desired, vpColVector& commanded) ;
// Move force point *legacy
void MeasuredForceFrameChange(vpColVector ip, vpColVector& sp, vpRotationMatrix sRi);
// Move Velocity point *legacy
void VelMovePoint(vpColVector Vt, vpColVector& Vg, vpHomogeneousMatrix bMg, vpHomogeneousMatrix bMt);
// Change force frame*legacy
void ForceFrameChange(vpColVector bF, vpColVector& gF, vpRotationMatrix bRg);

void GETCICuttingPoints(char* filename, int num, double**& data);

// Functions from Matlab/SYMORO

// Calculates the transformation matrix from gripper to object
void T70calc(vpColVector q, vpHomogeneousMatrix& T);
// Calculates the Jacobian matrix from gripper to object in Base frame
void J70calc(vpColVector q, vpMatrix& J);
void GETCIMaxVelAndAcc(char* filename, double* data);

void NormalR(vpMatrix r , vpMatrix& R);
void printfMp(vpMatrix M, char* s, int m, int n);
double VectorNorm(vpColVector v, int num);
void printfVector(vpColVector v, char* s, int m);


#endif
