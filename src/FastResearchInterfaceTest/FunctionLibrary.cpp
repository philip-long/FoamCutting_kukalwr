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
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPlane.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <fstream>

// OpenCv includes
#include "cv.h" 
#include "highgui.h" 
#include <stdio.h>  
#include "cvblobs/BlobResult.h"


#define PI 3.14159265358979

using namespace std;
/************************************************************************************************


First set of functions are those concerning data extraction from files

************************************************************************************************/

//get the Homogeneous Matrix(transformation matrix) from a file
void GETHomogeneousMatrix(vpHomogeneousMatrix& M,const char* filename)
{
  FILE *fp;  
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 
   for(int i=0;i<4;i++)
  { 
    fgets(Contentl, 100, fp);
    counter = 0;
    while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    for(int j=0;j<4;j++)
    { 
      k=0;
      while(Contentl[counter] != ' '&& Contentl[counter] != '\n' )
      { 
        pBuf[k] = Contentl[counter];
        k++;
        counter++;
      }
      M[i][j] = atof(pBuf);
      memset(pBuf,'\0',sizeof pBuf);
      counter++;
      while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    }
  }
}

//get the 7 gain parameters and the cycletime for joint position/impedance control
void GETParametersJ(char* filename, double* lamda, double& intertime)
{
  FILE *fp; 
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 

  fgets(Contentl, 100, fp);
  fgets(Contentl, 100, fp);
  counter = 0;
 /////////////////////////////////lamda
  for(int i=0;i<7;i++)
  {
  k=0;
  while(Contentl[counter] == ' ')
  { 
    counter++;
  }
  while(Contentl[counter] != ' '&& Contentl[counter] != '\n')
  { 
    pBuf[k] = Contentl[counter];
    k++;
    counter++;
  }
  lamda[i] = atof(pBuf);
  memset(pBuf,'\0',sizeof pBuf);
  }
//////////////////////////////////
  fgets(Contentl, 100, fp);
  counter = 0;
  k=0;
  while(Contentl[counter] == ' ')
  { 
    counter++;
  }
  while(Contentl[counter] != ' '&& Contentl[counter] != '\n')
  { 
    pBuf[k] = Contentl[counter];
    k++;
    counter++;
  }
  intertime = atof(pBuf);
  memset(pBuf,'\0',sizeof pBuf);

  return;
}

// Finds the transpose of the rotation matrix
void TranposeRotMat(vpRotationMatrix& Rt,vpRotationMatrix R)
{
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    Rt[i][j]=R[j][i];
                }
            }
}

void GETCIMaxVelAndAcc(char* filename, double* data)
{
  FILE *fp; 
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 

  fgets(Contentl, 100, fp);
  fgets(Contentl, 100, fp);
  counter = 0;
 /////////////////////////////////lamda
  for(int i=0;i<4;i++)
  {
  k=0;
  while(Contentl[counter] == ' ')
  { 
    counter++;
  }
  while(Contentl[counter] != ' '&& Contentl[counter] != '\n')
  { 
    pBuf[k] = Contentl[counter];
    k++;
    counter++;
  }
  data[i] = atof(pBuf);
  memset(pBuf,'\0',sizeof pBuf);
  }
//////////////////////////////////
 

  return;
}


//get the 6 gain parameters and the cycletime for cartesian impedance control
void GETParametersCI(const char* filename, double* lamda, double& intertime)
{
  FILE *fp; 
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 
  fgets(Contentl, 100, fp);
  fgets(Contentl, 100, fp);
  counter = 0;
 /////////////////////////////////lamda
  for(int i=0;i<6;i++)
  {
  k=0;
  while(Contentl[counter] == ' ')
  { 
    counter++;
  }
  while(Contentl[counter] != ' '&& Contentl[counter] != '\n')
  { 
    pBuf[k] = Contentl[counter];
    k++;
    counter++;
  }
  lamda[i] = atof(pBuf);
  memset(pBuf,'\0',sizeof pBuf);
  }
//////////////////////////////////
  fgets(Contentl, 100, fp);
  counter = 0;
  k=0;
  while(Contentl[counter] == ' ')
  { 
    counter++;
  }
  while(Contentl[counter] != ' '&& Contentl[counter] != '\n')
  { 
    pBuf[k] = Contentl[counter];
    k++;
    counter++;
  }
  intertime = atof(pBuf);
  memset(pBuf,'\0',sizeof pBuf);

  return;
}

//get the totle line number of a file
int GETFileLines(char* filename)
{
  FILE *fp; 
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  int hence;
  int num = 0;	
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 
  while(fgets(Contentl, 100, fp) != NULL)
  {
	num = num +1;
  }
  printf("file lines = %d\n", num);
  // num the total line
  // to read

  return num;
}

//get the trajectory of the robot, and store them in the data structure
//include: Px,Py,Pz,thetaUx,thetaUy,thetaUz,Fx,Fy,Fz,Omigax,Omigay,Omigaz
void GETCuttingPoints(char* filename, int num, double**& data)
{
  FILE *fp;  
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[100];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 
  fgets(Contentl, 100, fp);
   for(int i=0;i<num-1;i++)
  { 
    fgets(Contentl, 100, fp);
    counter = 0;
    while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    for(int j=0;j<12;j++)
    { 
      k=0;
      while(Contentl[counter] != ' '&& Contentl[counter] != '\n' )
      { 
        pBuf[k] = Contentl[counter];
        k++;
        counter++;
      }
      data[i][j] = atof(pBuf);
      memset(pBuf,'\0',sizeof pBuf);
      counter++;
      while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    }
  }
}


/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions are those concerning display

************************************************************************************************/



//  PRINT FUNCTIONS

void printfM(vpHomogeneousMatrix M)
{
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}


void printfM(vpHomogeneousMatrix M, char* s)
{
  printf("the vpHomogeneousMatrix matrix %s : \n", s);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
  printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}

void printfM(vpMatrix M)
{
  for(int i=0;i<M.getRows();i++)
  {
    for(int k=0;k<M.getCols();k++)
    {
	  printf("%8.3f ", M[i][k]);
    }
        printf("\n");
  }
}
void printfM(vpColVector v)
{
  for(int i=0;i<v.getRows();i++)
  {
    printf("%8.3f ", v[i]);
  }
    printf("\n");
}


//show the m by n matrix of transformation matrix
void printfMp(vpMatrix M, char* s, int m, int n)
{
  printf("the vpMatrix matrix %s : \n", s);

  for(int i=0;i<m;i++)
  {
    for(int k=0;k<n;k++)
    {
	  printf("%8.3f ", M[i][k]);
    }

        printf("\n");
  }

}


//show the vector of m dimension
void printfVector(vpColVector v, char* s, int m)
{
  printf("the vector value %s : \n", s);

  for(int i=0;i<m;i++)
  {
    printf("%8.3f ", v[i]);
  }
    printf("\n");
}



//show the 6 by 7 matrix of Jacobian matrix
void printfJ(vpMatrix J, char* s)
{
  printf("the Jacobian matrix %s :\n", s);
  for(int i=0;i<6;i++)
  {
	printf("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f \n", J[i][0], J[i][1], J[i][2], J[i][3], J[i][4], J[i][5], J[i][6]);
  }
}


//convert FRICartPose structure from FRI data structure, 4*3
void FRICartPose2vpHomogeneousMatrix(float* Pose,vpHomogeneousMatrix& M)
{
  for(int i=0;i<3;i++)
  {
	for(int j=0;j<4;j++)
	{
	  M[i][j] = Pose[i*4+j];
  	}
  }
  M[3][0]=0;
  M[3][1]=0;
  M[3][2]=0;
  M[3][3]=1;
  return;
}


/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/




/*------------------------------------------------------------------------------------
 
TransMat Returns a Homgeous Transformation matrix for an
 input of angle of translation vector

    choice=1 RX
    choice=2 RY
    choice=3 RZ
------------------------------------------------------------------------------------*/

void TransMat(int choice,float y,vpHomogeneousMatrix& T)
{

    switch(choice)
    {
        case 1:

            T[0][0]=1.0;T[0][1]=0.0;T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=cos(y);T[1][2]=-sin(y) ;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=sin(y);T[2][2]=cos(y);T[2][3]=0.0;
            break;
        
        case 2:

            T[0][0]=cos(y);T[0][1]=0.0;T[0][2]=sin(y);T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=1.0;T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=-sin(y);T[2][1]=0.0;T[2][2]=cos(y);T[2][3]=0.0;
            break;
        
        case 3:

            T[0][0]=cos(y);T[0][1]=-sin(y);T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=sin(y);T[1][1]=cos(y);T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=0.0;T[2][2]=1.0;T[2][3]=0.0;
            break;
        default:
            printf("Unrecognized returning identity");
            T[0][0]=1.0;T[0][1]=0.0;T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=1.0;T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=0.0;T[2][2]=1.0;T[2][3]=0.0;

    }
}


void TransRot(int choice,float y,vpRotationMatrix& R)
{

vpHomogeneousMatrix T;

  switch(choice)
    {
        case 1:

             T[0][0]=1.0;T[0][1]=0.0;T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=cos(y);T[1][2]=-sin(y) ;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=sin(y);T[2][2]=cos(y);T[2][3]=0.0;
            break;
        
        case 2:

            T[0][0]=cos(y);T[0][1]=0.0;T[0][2]=sin(y);T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=1.0;T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=-sin(y);T[2][1]=0.0;T[2][2]=cos(y);T[2][3]=0.0;
            break;
        
        case 3:

            T[0][0]=cos(y);T[0][1]=-sin(y);T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=sin(y);T[1][1]=cos(y);T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=0.0;T[2][2]=1.0;T[2][3]=0.0;
            break;
        default:
            printf("Unrecognized returning identity");
            T[0][0]=1.0;T[0][1]=0.0;T[0][2]=0.0;T[0][3]=0.0;
            T[1][0]=0.0;T[1][1]=1.0;T[1][2]=0.0;T[1][3]=0.0;
            T[2][0]=0.0;T[2][1]=0.0;T[2][2]=1.0;T[2][3]=0.0;

    }
    T.extract(R);

}
void TransMat(float* y,vpHomogeneousMatrix& T)
{

    T[0][0]=1.0;T[0][1]=0.0;T[0][2]=0.0;T[0][3]=y[0];
    T[1][0]=0.0;T[1][1]=1.0;T[1][2]=0.0;T[1][3]=y[1];
    T[2][0]=0.0;T[2][1]=0.0;T[2][2]=1.0;T[2][3]=y[2];

}


int sgnNumber(float x)
{
    float eps;
    eps=0.000001;
    return x<-eps?-1.0:x>eps;          
}

int sgnNumber(int x)
{
    int eps;
    eps=0;
    return x<-eps?-1:x>eps;    
}
/*------------------------------------------------------------------------------------

This finds the Cartesian Difference between two frame
------------------------------------------------------------------------------------*/


void CartesianDiff(vpHomogeneousMatrix& bMtold,vpHomogeneousMatrix& bMt,vpColVector& E)
{
vpRotationMatrix R;
vpHomogeneousMatrix Diff;

Diff=bMtold.inverse()*bMt;
Diff.extract(R);
    for (int i = 0; i < 3; i += 1)
    {
        E[i]=Diff[i][3];
    }

E[4]=0.5*(R[2][1]-R[1][2]);
E[5]=0.5*(R[0][2]-R[2][0]);
E[6]=0.5*(R[1][0]-R[0][1]);


}


/*------------------------------------------------------------------------------------

This finds the Caretsian Error between two matrix
------------------------------------------------------------------------------------*/


void CartesianError(vpHomogeneousMatrix& Td,vpHomogeneousMatrix& T,vpColVector& E)
{
vpRotationMatrix Rd,R,Rt,DiffMat;
vpTranslationVector Pd,P;
    
Td.extract(Pd);
Td.extract(Rd);
T.extract(P);
T.extract(R);
TranposeRotMat(Rt,R);
DiffMat=Rd*Rt;
    for (int i = 0; i < 3; i += 1)
    {
        E[i]=Pd[i]-P[i];
    }

E[4]=0.5*(DiffMat[2][1]-DiffMat[1][2]);
E[5]=0.5*(DiffMat[0][2]-DiffMat[2][0]);
E[6]=0.5*(DiffMat[1][0]-DiffMat[0][1]);


}
/************************************************************************************************


This set of functions are those concerning the conversion of the FRI JACOBIAN matrix
************************************************************************************************/



//Jacobian that we get from FRI is [Px,Py,Pz,thetauz,thetauy,thetaux], here we store it into a vpMatrix
//at the same time we change it into [Px,Py,Pz,thetaux,thetauy,thetauz] order
void FRIJaco2vpMatrix(float** fjaco,vpMatrix& M)
{
  for(int i=0;i<3;i++)
  {
	for(int j=0;j<7;j++)
	{
	  M[i][j] = fjaco[i][j];
  	}
  }

  for(int k=0;k<7;k++)
  {
    M[5][k] = fjaco[3][k];
    M[4][k] = fjaco[4][k];
    M[3][k] = fjaco[5][k];
  }

}


/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions are those concerning vector operations

************************************************************************************************/



//find the max absolute value of a vector
double findabsmax(vpColVector v,int n)
{
  double max;
  max = fabs(v[0]);
  for(int i=1;i<n;i++)  {
   if(max < fabs(v[i]))
     max = fabs(v[i]);
  } 
  return max;
} 

//get the norm of a vector
double VectorSqrt(vpColVector v, int num)
{
  double sum=0;
  for(int i=0;i<num;i++)
  {
    sum = sum + v[i]*v[i];
  }
  sum = sum/num;
  return sqrt(sum);
}

double VectorNorm(vpColVector v, int num)
{
  double sum=0;
  for(int i=0;i<num;i++)
  {
    sum = sum + v[i]*v[i];
  }
  sum = sum;
  return sqrt(sum);
}


//put two 3 dimension vector to one 6 dimension vector
//put two 3 dimension vector to one 6 dimension vector
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpRzyxVector b, vpColVector& c)
{
  c[0]=a[0];
  c[1]=a[1];
  c[2]=a[2];
  c[3]=b[0];
  c[4]=b[1];
  c[5]=b[2];
}

//put two 3 dimension vector to one 6 dimension vector
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpRzyzVector b, vpColVector& c)
{
  c[0]=a[0];
  c[1]=a[1];
  c[2]=a[2];
  c[3]=b[0];
  c[4]=b[1];
  c[5]=b[2];
}

//put two 3 dimension vector to one 6 dimension vector
void Two3dimensionVector2OnedemensionVector( vpTranslationVector a, vpThetaUVector b, vpColVector& c)
{
  c[0]=a[0];
  c[1]=a[1];
  c[2]=a[2];
  c[3]=b[0];
  c[4]=b[1];
  c[5]=b[2];
}

/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions relates to changing of the frame of velocity, force, jacobian etc.

************************************************************************************************/



//Changes the frame of the Jacobian matrix
void JacobianFrameChange(vpMatrix iJ, vpMatrix& sJ, vpRotationMatrix sRi)
{
  vpMatrix trans;
  trans.eye(6);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans[i][j] = sRi[i][j];
      trans[i+3][j+3] = sRi[i][j];
    }
  } 
  sJ = trans*iJ;
} 


//get the force at the same point but wrt to another fame
void ForceFrameChange(vpColVector bF, vpColVector& gF, vpRotationMatrix bRg)
{
  vpMatrix trans;
  trans.eye(6);
  vpRotationMatrix gRb;
  gRb=bRg.inverse();
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans[i][j] = gRb[i][j];
      trans[i+3][j+3] = gRb[i][j];
    }
  } 
  gF = trans*bF;
} 


void MeasuredForceFrameChange(vpColVector ip, vpColVector& sp, vpRotationMatrix sRi)
{
  vpMatrix trans;
  trans.eye(6);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans[i][j] = sRi[i][j];
      trans[i+3][j+3] = sRi[i][j];
    }
  } 
  sp = trans*ip;
} 



/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions are those concerning the measuing point of the velocity, force, Jacobian

************************************************************************************************/


//move the represent point of the Jacobian to another point, wrt the same frame; the base is usually the base frame
void JacobianMovePoint(vpMatrix Jg, vpMatrix& Jt, vpHomogeneousMatrix bMg, vpHomogeneousMatrix bMt)
{
  vpColVector p(3);
  for(int i=0;i<3;i++)
    p[i] = bMt[i][3]-bMg[i][3];
   
  vpMatrix skew;
  skew.eye(3);
  SkewSym(p, skew);  

  vpMatrix x;
  x.eye(6);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      x[i][j+3] = -skew[i][j];
    }
  } 

  Jt = x*Jg;
} 








//get the velocity of a another point in the same body
void VelMovePoint(vpColVector Vt, vpColVector& Vg, vpHomogeneousMatrix bMg, vpHomogeneousMatrix bMt)
{
  vpColVector p(3);
  for(int i=0;i<3;i++)
    p[i] = bMg[i][3]-bMt[i][3];
   
  vpMatrix skew;
  skew.eye(3);
  SkewSym(p, skew);  

  vpMatrix x;
  x.eye(6);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      x[i][j+3] = -skew[i][j];
    }
  } 

  Vg = x*Vt;
} 



/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions are those concerning Qi's control law

************************************************************************************************/

//get qdot from the control law
void ComputeControlLaw(vpColVector& qdot, vpMatrix Jaco_base, vpMatrix L, double* lamda, vpColVector deltaS)
{
  vpMatrix LJ(6,7);
  LJ = L*Jaco_base;
  //compute the qdot
  qdot = (LJ.pseudoInverse())*deltaS;
  for(int i=0;i<7;i++)
  {
  qdot[i]=-lamda[i]*qdot[i];
  } 

} 

//get the gradient of the secondary task, joint avoidance
void JointLimitAvoid(float* q, vpColVector& z)
{
  float jointup[7];
  float jointlow[7];
  float jointup1[7];
  float jointlow1[7];
  float jointtolerence[7];
  float delta[7];
  double scale = 1;

  jointup[0]=170;
  jointup[1]=120;
  jointup[2]=170;
  jointup[3]=120;
  jointup[4]=170;
  jointup[5]=120;
  jointup[6]=170;
  jointlow[0]=-170;
  jointlow[1]=-120;
  jointlow[2]=-170;
  jointlow[3]=-120;
  jointlow[4]=-170;
  jointlow[5]=-120;
  jointlow[6]=-170;  
  for(int i=0;i<7;i++)
  {
     jointtolerence[i] = 20;
     jointup1[i]=jointup[i]-scale*jointtolerence[i];    
     jointlow1[i]=jointlow[i]+scale*jointtolerence[i];  
     delta[i] = jointup[i]-jointlow[i];
     if(q[i]<jointlow1[i])
     {
	   z[i]=(q[i]-jointlow1[i])/delta[i];
     }
     else if(q[i]>jointup1[i])
     {
       z[i]=(q[i]-jointup1[i])/delta[i];
     }
     else
     {
        z[i] = 0;
     }
  }
} 

//get qdot from the control law, at the same time using the classic projector to excute the secondary task
void ComputeControlLawSecondTask(vpColVector& qdot, vpMatrix Jaco_base, vpMatrix L, double* lamda, vpColVector deltaS, vpColVector z)
{
  vpMatrix LJ(6,7);
  vpMatrix LJRest(7,7);
  vpMatrix I;
  I.eye(7);
  LJRest = I-Jaco_base.pseudoInverse()*Jaco_base;
  LJ = L*Jaco_base;
  //compute the qdot
  qdot =((LJ.pseudoInverse())*deltaS+LJRest*z);
  for(int i=0;i<7;i++)
  {
  qdot[i]=-lamda[i]*qdot[i];
  } 
} 

//get v of P and thetaU from control law
void ComputeControlLawCartesianImpedance(vpColVector& v, vpMatrix L, double* lamda, vpColVector deltaS)
{
  //compute the qdot
  v = (L.pseudoInverse())*deltaS;

  for(int i=0;i<6;i++)
  {
  v[i]=-lamda[i]*v[i];
  } 
} 



//to make the qdot continueous and not exceed the default maximum
void Selectqdot(vpColVector& after, vpColVector cal,vpColVector before, double t)
{
  double MaxVel = 10.0;
  double MaxAcc = 10.0;
  double max;
  max = findabsmax(cal,7);
  vpColVector calnormal(7);

  if(max>MaxVel)
    calnormal = (MaxVel/max)*cal;
  else 
    calnormal = cal;

  vpColVector delta(7);
  for(int i=0;i<7;i++)
  {
  delta[i] = calnormal[i]-before[i];
  }
  for(int i=0;i<7;i++)
  {
   if(fabs(delta[i]) < MaxAcc*t)
   {
     after[i] = calnormal[i];
   }
   else
   {
     if(delta[i]<0)
     {
     after[i] = before[i]-MaxAcc*t;
     }
     else
     {
     after[i] = before[i]+MaxAcc*t;
     }
   }  
 } 
} 

//to make the v continueous and not exceed the default maximum
void Selectxdot(vpColVector& after, vpColVector cal, vpColVector before, double t, double* data)
{

  double MaxAccr = data[3];
  double MaxAccx = data[2];
  double MaxVelr = data[1];
  double MaxVelx = data[0];
  double maxv;
  double maxr;
  vpColVector x(3);
  vpColVector r(3);
  vpColVector delta(6);
  vpColVector normal(6);


  for(int i=0;i<3;i++)
  {
  x[i] = cal[i];
  r[i] = cal[i+3];
  }
  maxv = findabsmax(x,3);
  maxr = findabsmax(r,3);

  if(maxv>MaxVelx)
    x = (MaxVelx/maxv)*x;

  if(maxr>MaxVelr)
    r = (MaxVelr/maxr)*r;

  for(int i=0;i<3;i++)
  {
  normal[i] = x[i];
  normal[i+3] = r[i];
  }
  delta = normal-before;

  for(int i=0;i<3;i++)
  {
   if(fabs(delta[i]) < MaxAccx*t)
     after[i] = normal[i];
   else
   {
     if(delta[i]<0)
     {
     after[i] = before[i]-MaxAccx*t;
     }
     else
     {
     after[i] = before[i]+MaxAccx*t;
     }
   }  
  if(fabs(delta[i+3]) < MaxAccr*t)
     after[i+3] = normal[i+3];
   else
   {
     if(delta[i+3]<0)
     {
     after[i+3] = before[i+3]-MaxAccr*t;
     }
     else
     {
     after[i+3] = before[i+3]+MaxAccr*t;
     }
   }  
 } 
} 

//get the matrix Lw; Lw matrix is the component of L
//Lw is the transformation of thetaU_dot to w
void ComputeLw(vpColVector thetau, vpMatrix& Lw) 
{
  double theta;
  vpColVector u(3);
  theta = sqrt(pow(thetau[0],2)+pow(thetau[1],2)+pow(thetau[2],2));
  u[0]=thetau[0]/theta;
  u[1]=thetau[1]/theta;
  u[2]=thetau[2]/theta;
//
  vpMatrix I(3,3);
  vpMatrix uskewsym(3,3);
  I.eye(3);
  SkewSym(u,uskewsym);
  Lw = I-theta/2*uskewsym+(1-computesinc(theta)/pow(computesinc(theta/2),2))*uskewsym*uskewsym;

}

void SkewSym(vpColVector u, vpMatrix& L) 
{
  L[0][0] = 0;
  L[0][1] = -u[2];
  L[0][2] = +u[1];
  L[1][0] = u[2];
  L[1][1] = 0;
  L[1][2] = -u[0];
  L[2][0] = -u[1];
  L[2][1] = u[0];
  L[2][2] = 0;
}


//get the sinc(theta)
double computesinc(double theta)
{
  if(fabs(theta<0.001))
    return 1;
  else
    return sin(theta)/theta;
}

//when camera fixed, get the L matrix only consider the Position 
//which means finally only position will reach the desired position *********debug***********
void LPositionFixCam(vpHomogeneousMatrix bmo, vpMatrix& c)
{

  vpMatrix mb,mm;
  vpMatrix ma(3,6);

  vpHomogeneousMatrix omb;
  omb = bmo.inverse();

  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      ma[i][j] = omb[i][j];
      ma[i][j+3] = 0;
    }
  } 

  mm.eye(3);
  mm[0][0] = 0;
  mm[0][1] = -bmo[2][3];
  mm[0][2] = -bmo[1][3];
  mm[1][0] = bmo[2][3];
  mm[1][1] = 0;
  mm[1][2] = -bmo[0][3];
  mm[2][0] = bmo[1][3];
  mm[2][1] = bmo[0][3];
  mm[2][2] = 0;
  
  mb.eye(6);
  mb[0][0] = 1;
  mb[1][1] = 1;
  mb[2][2] = 1;


  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      mb[i][j+3] = -mm[i][j];
    }
  }
  
  c = ma*mb;

}

//when camera fixed, get the L matrix  
//which means finally the pose will reach the desired pose
void LPoseFixCam(vpHomogeneousMatrix bmg,vpHomogeneousMatrix bmo, vpMatrix& c)
{

  vpHomogeneousMatrix omb,omg;
  omb = bmo.inverse();
  omg = omb*bmg;

  vpRotationMatrix org,orb;
  vpThetaUVector othetaug;
  vpTranslationVector bto;
  omg.extract(org);
  omb.extract(orb);
  bmo.extract(bto);
  othetaug.buildFrom(org);


  vpMatrix lw,l1,l2,skewt;
  lw.eye(3);
  skewt.eye(3);
  SkewSym(bto,skewt);
  ComputeLw(othetaug,lw);

  l1.eye(6);
  l2.eye(6);

  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      l1[i][j] = orb[i][j];
      l1[i+3][j+3] = orb[i][j];
    }
  } 

 
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      l2[i+3][j+3] = lw[i][j];
    }
  }
  
  //c = l2*l1;
  c=l1;

}




//get the num of eigen value of a matrix *********not working***********
int EigenValueNum(vpMatrix J)
{
  vpMatrix Jt(7,6);
  vpMatrix JJt(6,6);
  Jt = J.t();
  JJt = J*Jt;

  vpColVector eigenvalues(6);
  for(int i=0;i<6;i++)
    eigenvalues[i] = 0;
  
  eigenvalues=JJt.eigenValues();
  int num = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(eigenvalues[i]) > 0.01)
      num++;
  }
    
  return num;
  
}

//to get the average transformation matrix
//this is because when ViSP track the object, there is noise, so we filter the matrix 
vpHomogeneousMatrix Filter( vpHomogeneousMatrix cMoa,  vpHomogeneousMatrix cMob,  vpHomogeneousMatrix cMobb)
{
  double a,b,c;
  a = 5;
  b = 2;
  c = 1;
  vpHomogeneousMatrix cMo;

  for(int i=0;i<4;i++)
  {
  for(int j=0;j<4;j++)
  {
    cMo[i][j]=(a*cMoa[i][j]+b*cMob[i][j]+c*cMobb[i][j])/(a+b+c);

  }
  }
  return cMo;

}


//a simple force contrl law using force feedback, not sure if it's good or not
void ComputeCommandedForce(vpColVector measured, vpColVector desired, vpColVector& commanded) 
{
  double lamda = 0.2;
  // the measured force is in world frame, force on the sensor
  // the desired force is in world frame, force on object

    commanded[2] = desired[2]+lamda*(desired[2]-measured[2]);
  
}










/*

Force Sensor

*/



/* This function, ZeroForceSensor, take the force reading and cancels 

(a) Electronic Bias
(b) Force due to sensor's own weight
(c) Force due to tool weight

Finally the force is converted to the TCP
*/
void ZeroForceSensor(vpHomogeneousMatrix bMt,float* Measuredforce,vpColVector& ResolvedForce)
{

	double ComMx=0.0;
	double ComMy=0.0;
	double ComMz=0.12; // Centre of mass to tool frame
	double Mx=0.0;
	double My=0.0;
	double Mz=0.14; // Tool Frame Offset
	float Mass=0.2; // Mass of Tool 0.3
	float MassForceSensor=0.09	; // Mass of Flexible type of Sensor


	float Bias[6];
	float Mforce[6];
	for(int i=0;i<6;i++) 
	{
	Mforce[i]=Measuredforce[i]; // Assigning to a local variable
	}

	
	vpMatrix LC,LD,L1,L2,L3;L1.eye(6);L2.eye(6);L3.eye(6); // Blocks of the interaction matrix

	//vpHomogeneousMatrix bMt;// Transformation matrix tool to base
	vpHomogeneousMatrix fMt,tMf; // Transformation matrix tool to force sensor
	vpHomogeneousMatrix bMf; // Transformation matrix from force sensor to base
	vpRotationMatrix bRt,tRb; // Rotation matrix tool to base
	vpRotationMatrix fRt,tRf; // Rotation matrix tool to force sensor
	vpRotationMatrix fRb; // Rotation matrix base to force
	
	vpTranslationVector fTt,tTf,tcomTt; // Translation vector tool to force
	vpMatrix tTf_skew(3,3),tcomTt_skew(3,3); // skew
	vpColVector fFf(6),tFt(6); 
	vpColVector fFf_mass(6); // Wrench due to weight of sensor at sensor
	vpColVector tFt_mass(6);
	vpColVector bFf_mass(6); // Wrench due to weight of sensor in base
	vpColVector bFt_mass(6); // Wrench due to weight of tool in base

	// Define Bias as averge of found bias
	Bias[0]=9.26;
	Bias[1]=-0.92;
	Bias[2]=-1.79;//-1.19
	Bias[3]=-0.0065;
	Bias[4]=-1.435;
	Bias[5]=-0.014;

		// Force due to mass of tool
	bFt_mass[0]=0;
	bFt_mass[1]=0;
	bFt_mass[2]=-9.81*Mass;
	bFt_mass[3]=0;
	bFt_mass[4]=0;
	bFt_mass[5]=0;

	// Force due to mass of sensor
	bFf_mass[0]=0;
	bFf_mass[1]=0;
	bFf_mass[2]=-9.81*MassForceSensor;
	bFf_mass[3]=0;
	bFf_mass[4]=0;
	bFf_mass[5]=0;


	// This is the transofrmation matrix from tool to force sensor
	fMt[0][0]=-0.0;
	fMt[1][0]=-1.0 ;
	fMt[2][0]=0.00;
	fMt[3][0]=0.000 ;
	fMt[0][1]=1.0;
	fMt[1][1]= 0.00; 
	fMt[2][1]=0.00 ;
	fMt[3][1]=0.000;
	fMt[0][2]=0.00 ;
	fMt[1][2]=0.00 ;
	fMt[2][2]=1.0;
	fMt[3][2]=0.000;
	fMt[0][3]=Mx;
	fMt[1][3]=My;
	fMt[2][3]=Mz;
	fMt[3][3]=1.0;


	tMf=fMt.inverse();
	tMf.extract(tRf);
	fMt.extract(fRt);

	tMf.extract(tTf);
	SkewSym(tTf, tTf_skew);


	bMt.extract(bRt);
	tRb=bRt.t();
	bMf=bMt*(tMf);
	fRb=fRt*(bRt.t());

	tcomTt[0]=ComMx;
	tcomTt[1]=ComMy;
	tcomTt[2]=ComMz;
	SkewSym(tcomTt, tcomTt_skew);

	/*

		 Build the Three L matrices
		(1) L1 converts the effect of the sensor mass from the world frame to the sensor frame
		(2) L2 converts the sensed force (after conversion and minus the bias and mass) from fFf to tFt, i.e.
			from the force sensor origin in the force sensor frame to the tool origin in the tool frame
		(3) L3 converts the effect of the mass of the tool from the world frame to the tool frame
		
	*/


	// L1=[ fRb 0 ; 0 fRb] 


	//printf("\n Measured force :=[%8.3f , %8.3f , %8.3f ,  %8.3f , %8.3f , %8.3f] \n",Measuredforce[0],Measuredforce[1],Measuredforce[2],Measuredforce[3],Measuredforce[4],Measuredforce[5]);
	
	for (int i=0;i<3;i++)
	{			
		for(int j=0;j<3;j++)
		{
			L1[i][j]=fRb[i][j]; // Upper  left
			L1[i][j+3]=0;// Upper right  
			L1[i+3][j]=0;// Lower left 
			L1[i+3][j+3]=fRb[i][j];// Lower right
		}		
	}

	fFf_mass=(L1*bFf_mass); // Weight of force sensor to force sensor frame

	for(int i=0;i<6;i++) // Convert units and eliminate the bias
	{
		  Mforce[i]=Mforce[i]/1000000;
		  Mforce[i]=Mforce[i]-Bias[i];
		  Mforce[i]=Mforce[i]-fFf_mass[i];
		  fFf[i]=Mforce[i];
	}
	
	//printf("\n Measured force less bias and mass in force sensor frame:=[%8.3f , %8.3f , %8.3f ,  %8.3f , %8.3f , %8.3f] \n",fFf[0],fFf[1],fFf[2],fFf[3],fFf[4],fFf[5]);

	// L2= [ tRf 0 ; -skew(fTf)*tRf  tRf]


	LC=-tTf_skew*tRf;
	
	for (int i=0;i<3;i++)
	{			
		for(int j=0;j<3;j++)
		{
			L2[i][j]=tRf[i][j]; // Upper  left
			L2[i][j+3]=0;// Upper right  
			L2[i+3][j]=LC[i][j];// Lower left 
			L2[i+3][j+3]=tRf[i][j];// Lower right
		}		
	}
		
	tFt=(L2*fFf); // Measured force at Tool frame
	//printf("\n Resolved force at TCP with mass of tool:=[%8.3f ,%8.3f , %8.3f , %8.3f , %8.3f , %8.3f] \n",tFt[0],tFt[1],tFt[2],tFt[3],tFt[4],tFt[5]);

	// L3=[ tRb 0 ; -skew(tcomTt)*tRb tRb]
	
	LC=tcomTt_skew*tRb;
	
	for (int i=0;i<3;i++)
	{			
		for(int j=0;j<3;j++)
		{
			L3[i][j]=tRb[i][j]; // Upper  left
			L3[i][j+3]=0;// Upper right  
			L3[i+3][j]=LC[i][j];// Lower left 
			L3[i+3][j+3]=tRb[i][j];// Lower right
		}		
	}
	
	tFt_mass=L3*bFt_mass; // Weight of Tool in Tool frame
	
	//printf("\n Tool Weight at TCP:=[%8.3f,  %8.3f ,%8.3f ,  %8.3f , %8.3f , %8.3f] \n",tFt_mass[0],tFt_mass[1],tFt_mass[2],ResolvedForce[3],tFt_mass[4],tFt_mass[5]);


	for(int i=0;i<6;i++) // Convert units and eliminate the bias
	{
	ResolvedForce[i]=tFt[i]-tFt_mass[i];
	}

	//printf("\n Resolved force at TCP:=[%8.3f , %8.3f,  %8.3f  , %8.3f, %8.3f , %8.3f] \n",ResolvedForce[0],ResolvedForce[1],ResolvedForce[2],ResolvedForce[3],ResolvedForce[4],ResolvedForce[5]);


}

//***********************************************************************************************************

void ZeroForceSensorTest(vpHomogeneousMatrix bMt,float* Measuredforce2,vpColVector& ResolvedForce2)
{

	double ComMx=0.0;
	double ComMy=0.0;
	double ComMz=0.02; // Centre of mass to force sensor
	double Mx=0.0;
	double My=0.0;
	double Mz=0.14; // Tool Frame Offset
	float Mass=0.2; // Mass of Tool
	float MassForceSensor=0.09	; // Mass of Flexible type of Sensor
	vpTranslationVector tcomTt; // Translation vector tool to force
	tcomTt[0]=ComMx;
	tcomTt[1]=ComMy;
	tcomTt[2]=ComMz;


	vpMatrix tcomTt_skew(3,3); // skew

	float Bias[6];
	float Mforce2[6];
	for(int i=0;i<6;i++) 
	{
	Mforce2[i]=Measuredforce2[i]; // Assigning to a local variable
	}


	

	vpMatrix LA,LB,LC,LC2,LD,L,L2,L3;L.eye(6);L2.eye(6);L3.eye(6); // Blocks of the interaction matrix
	//vpHomogeneousMatrix bMt;// Transformation matrix tool to base
	vpHomogeneousMatrix fMt; // Transformation matrix tool to force sensor
	vpHomogeneousMatrix bMf;
	vpRotationMatrix bRt; // Rotation matrix tool to base
	vpRotationMatrix fRt,tRf; // Rotation matrix tool to force sensor
	vpRotationMatrix fRb;
	
	vpTranslationVector fTt;
	vpMatrix fTt_skew(3,3);
	vpColVector fFt(6);
	vpColVector fFf_mass(6);
	vpColVector bFf_mass(6);
	vpColVector bFt_mass(6);

	// Define Bias as averge of found bias
	Bias[0]=9.26;
	Bias[1]=-0.92;
	Bias[2]=-1.79;
	Bias[3]=-0.0065;
	Bias[4]=-1.435;
	Bias[5]=-0.014;


		// Force due to mass of tool
	bFt_mass[0]=0;
	bFt_mass[1]=0;
	bFt_mass[2]=-9.81*Mass;
	bFt_mass[3]=0;
	bFt_mass[4]=0;
	bFt_mass[5]=0;

	// Force due to mass of sensor
	bFf_mass[0]=0;
	bFf_mass[1]=0;
	bFf_mass[2]=-9.81*MassForceSensor;
	bFf_mass[3]=0;
	bFf_mass[4]=0;
	bFf_mass[5]=0;


	// This is the transofrmation matrix from tool to force sensor
	fMt[0][0]=-0.0;
	fMt[1][0]=-1.0 ;
	fMt[2][0]=0.00;
	fMt[3][0]=0.000 ;
	fMt[0][1]=1.0;
	fMt[1][1]= 0.00; 
	fMt[2][1]=0.00 ;
	fMt[3][1]=0.000;
	fMt[0][2]=0.00 ;
	fMt[1][2]=0.00 ;
	fMt[2][2]=1.0;
	fMt[3][2]=0.000;
	fMt[0][3]=Mx;
	fMt[1][3]=My;
	fMt[2][3]=Mz;
	fMt[3][3]=1.000;
	fMt.extract(fRt);// extract the rotation matrix
	fMt.extract(fTt);//extract the deplacement vector
	SkewSym(fTt, fTt_skew);
	SkewSym(tcomTt, tcomTt_skew);

	bMt.extract(bRt);
	bMf=bMt*(fMt.inverse());

		
		fRb=fRt*(bRt.t());
		tRf=fRt.transpose();		

		LA=fRb;
		LC=-tcomTt_skew*fRb;
		LC2=fTt_skew*tRf;
		LD=fRb;
	
		for (int i=0;i<3;i++)
		{			
			for(int j=0;j<3;j++)
			{

				L[i][j]=LA[i][j]; // Upper  left 
				L2[i][j]=LA[i][j]; // Upper  left
				L3[i][j]=tRf[i][j]; // Upper  left	  

				L[i+3][j+3]=LD[i][j];// Lower right
				L2[i+3][j+3]=LD[i][j];// Lower right
				L3[i+3][j+3]=tRf[i][j];// Lower right

				L[i][j+3]=0;// Upper right 
				L2[i][j+3]=0;// Upper right
				L3[i][j+3]=0;// Upper right

				L[i+3][j]=LC[i][j];// Lower left 
				L2[i+3][j]=0;// Lower left 
				L3[i+3][j]=LC2[i][j];// Lower left 

			}		
		}

   	//	printf("\nFUNC2: Measured force :=[%8.3f , %8.3f , %8.3f ,  %8.3f , %8.3f , %8.3f] \n",Measuredforce2[0],Measuredforce2[1],Measuredforce2[2],Measuredforce2[3],Measuredforce2[4],Measuredforce2[5]); 
	

	fFf_mass=(L2*bFf_mass); // Weight of force sensor to force sensor frame
	fFt=L*bFt_mass; // Weight of Tool to force sensor frame


	for(int i=0;i<6;i++)
	{
		  Mforce2[i]=Mforce2[i]/1000000;
		  Mforce2[i]=Mforce2[i]-Bias[i];
		  Mforce2[i]=Mforce2[i]-fFf_mass[i];
	}
		
	//printf("\n FUNC2: Measured force less bias and mass in force sensor frame:=[%8.3f, %8.3f , %8.3f, %8.3f,%8.3f,%8.3f] \n",Mforce2[0],Mforce2[1],Mforce2[2],Mforce2[3],Mforce2[4],Mforce2[5]);



		for(int i=0;i<6;i++)
	{
		  Mforce2[i]=Mforce2[i]-fFt[i];
			ResolvedForce2[i]=Mforce2[i];
	}
	

	//printf("\n FUNC2: Resolved force  at Force Sensor Frame:=[%8.3f  %8.3f  %8.3f  %8.3f %8.3f %8.3f] \n",ResolvedForce2[0],ResolvedForce2[1],ResolvedForce2[2],ResolvedForce2[3],ResolvedForce2[4],ResolvedForce2[5]);



	// Transform to tool frame 
	ResolvedForce2=L3*ResolvedForce2;
	//printf("\n FUNC2: Resolved force  at TCP:=[%8.3f  %8.3f  %8.3f %8.3f  %8.3f %8.3f] \n",ResolvedForce2[0],ResolvedForce2[1],ResolvedForce2[2],ResolvedForce2[3],ResolvedForce2[4],ResolvedForce2[5]);
}


/*
				FUNCTION: ResolveForceatTCP
//******************************************************************************************************************************************************
Inputs: 1. Measured Force (raw data from force sensor), 
		2. bMt (the transformation matrix from TCP to base)
		3. eMf the [constant] transformation matrix from force sensor to end effector, 
		4. eMt the [constant] transformation matrix from tool to the end effetor
		5. x the identified paremeters
Outputs:  The resolved force at the TCP such that when there is no contact the force = 0

Summary: The function uses a matrix W found by GenerateW.m and values x found from GenerateX.m
to calculate the forces due to tool weight bias and force sensor weight at TCP. These forces are then subtracted from the measured forces
at TCP to get the NET contact forces. W and X must be recalculated for a different tool.


/home/kuka/FRILibrary/MinForceSensor/Linux/x86/debug/bin/MatlabFiles/


Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************


*/
void ResolveForceatTCP(vpHomogeneousMatrix bMt,vpHomogeneousMatrix eMf,vpHomogeneousMatrix eMt,float* Measuredforce2,vpColVector& ResolvedForce2,vpColVector x)
{


// Declare all force related variables
//**********************************************************************************************************
	vpColVector F_measured(6); // The measured force at force sesnor point
	vpColVector Ftcp_measured(6); // The measured force at TCP point
	vpColVector EstimatedForces(6); // Estimated force at tool point
//**********************************************************************************************************

// Declare matricial variables for calculation
//**********************************************************************************************************

	// X contains the identified values of the following
	// [bias MassForceSensor MassofTool*Sx MassofTool*St MassofTool*Sz]

	vpMatrix LC,L;
	L.eye(6);LC.eye(3); // Blocks of the matrix that transforms measured forces to TCP
	vpMatrix W(6,11); // Matrix that relates X to Forces at TCP !!!!!FOUND OFFLINE!!!!!
	vpMatrix skewfPt(3,3);// SkewSymmetric matrix
	float M_t=1.0; // The mass which is independently unknown

//**********************************************************************************************************


// Declare all homogenous, rotation and translational matrices and vectors
//**********************************************************************************************************

vpHomogeneousMatrix tMf,fMt,tMb,fMb;

    vpRotationMatrix fRt;
    vpRotationMatrix tRf;
	vpTranslationVector fPt;
	tMf=(eMt.inverse())*eMf;
	fMt=tMf.inverse();
	tMb=bMt.inverse();
	fMb=fMt*tMb;
	fMt.extract(fRt);
	tMf.extract(tRf);		

	//tRf=fRt.transpose();		


	fMt.extract(fPt);//extract the deplacement vector
	SkewSym(fPt, skewfPt);
	

	float tMf11=tMf[0][0]; float tMf12=tMf[0][1]; float tMf13=tMf[0][2]; float fMt14=tMf[0][3];
	float tMf21=tMf[1][0]; float tMf22=tMf[1][1]; float tMf23=tMf[1][2]; float fMt24=tMf[1][3];
	float tMf31=tMf[2][0]; float tMf32=tMf[2][1]; float tMf33=tMf[2][2]; float fMt34=tMf[2][3];

	float	fMt11=fMt[0][0]; float fMt12=fMt[0][1]; float fMt13=fMt[0][2]; fMt14=fMt[0][3];
	float	fMt21=fMt[1][0]; float fMt22=fMt[1][1]; float fMt23=fMt[1][2]; fMt24=fMt[1][3];
	float	fMt31=fMt[2][0]; float fMt32=fMt[2][1]; float fMt33=fMt[2][2]; fMt34=fMt[2][3];

	float   fMb11=fMb[0][0]; float fMb12=fMb[0][1]; float fMb13=fMb[0][2];
	float   fMb21=fMb[1][0]; float fMb22=fMb[1][1]; float fMb23=fMb[1][2];
	float   fMb31=fMb[2][0]; float fMb32=fMb[2][1]; float fMb33=fMb[2][2];

	float tMb11=tMb[0][0]; float tMb12=tMb[0][1]; float tMb13=tMb[0][2];
	float tMb21=tMb[1][0]; float tMb22=tMb[1][1]; float tMb23=tMb[1][2];
	float tMb31=tMb[2][0]; float tMb32=tMb[2][1]; float tMb33=tMb[2][2];


//***********************************************************************************************************

	
// Step 1. Transform Raw Measured Forces to TCP frame
	for(int i=0;i<6;i++) 
	{
		F_measured[i]=Measuredforce2[i]/1000000; // Assigning to a local variable and converting to N, Nm
	}


	LC=tRf*skewfPt;	// minus sign is placed in for loop for syntax reasons

 for (int i=0;i<3;i++)
	{			
		for(int j=0;j<3;j++)
		{
			L[i][j]=tRf[i][j]; // Upper  left 
			L[i][j+3]=0;// Upper right 
			L[i+3][j]=LC[i][j];// Lower left 
			L[i+3][j+3]=tRf[i][j];// Lower right
		}		
	}
	//printfMp(L, "L=", 6, 6);
	Ftcp_measured=L*F_measured; // Weight of Tool to force sensor frame

	//printf("\n Ftcp_measured:=[%8.3f  %8.3f  %8.3f %8.3f  %8.3f %8.3f] \n",Ftcp_measured[0],Ftcp_measured[1],Ftcp_measured[2],Ftcp_measured[3],Ftcp_measured[4],Ftcp_measured[5]);

// Step 2. Calculated Estimated forces due to weight of tool etc.

/*------------------------------------------------------ 
 Matrix Generated by function Mat2Cmatrix 
------------------------------------------------------ */ 
 
W[0][0]=tMf11;
W[0][1]=tMf12;
W[0][2]=tMf13;
W[0][3]=0;
W[0][4]=0;
W[0][5]=0;
W[0][6]=- (981*fMb13*tMf11)/100 - (981*fMb23*tMf12)/100 - (981*fMb33*tMf13)/100;
W[0][7]=0;
W[0][8]=0;
W[0][9]=0;
W[0][10]=-(981*tMb13)/100;
W[1][0]=tMf21;
W[1][1]=tMf22;
W[1][2]=tMf23;
W[1][3]=0;
W[1][4]=0;
W[1][5]=0;
W[1][6]=- (981*fMb13*tMf21)/100 - (981*fMb23*tMf22)/100 - (981*fMb33*tMf23)/100;
W[1][7]=0;
W[1][8]=0;
W[1][9]=0;
W[1][10]=-(981*tMb23)/100;
W[2][0]=tMf31;
W[2][1]=tMf32;
W[2][2]=tMf33;
W[2][3]=0;
W[2][4]=0;
W[2][5]=0;
W[2][6]=- (981*fMb13*tMf31)/100 - (981*fMb23*tMf32)/100 - (981*fMb33*tMf33)/100;
W[2][7]=0;
W[2][8]=0;
W[2][9]=0;
W[2][10]=-(981*tMb33)/100;
W[3][0]=fMt24*tMf13 - fMt34*tMf12;
W[3][1]=fMt34*tMf11 - fMt14*tMf13;
W[3][2]=fMt14*tMf12 - fMt24*tMf11;
W[3][3]=tMf11;
W[3][4]=tMf12;
W[3][5]=tMf13;
W[3][6]=(981*fMb23*(fMt14*tMf13 - fMt34*tMf11))/100 - (981*fMb33*(fMt14*tMf12 - fMt24*tMf11))/100 - (981*fMb13*(fMt24*tMf13 - fMt34*tMf12))/100;
W[3][7]=0;
W[3][8]=(981*tMb33)/100;
W[3][9]=-(981*tMb23)/100;
W[3][10]=0;
W[4][0]=fMt24*tMf23 - fMt34*tMf22;
W[4][1]=fMt34*tMf21 - fMt14*tMf23;
W[4][2]=fMt14*tMf22 - fMt24*tMf21;
W[4][3]=tMf21;
W[4][4]=tMf22;
W[4][5]=tMf23;
W[4][6]=(981*fMb23*(fMt14*tMf23 - fMt34*tMf21))/100 - (981*fMb33*(fMt14*tMf22 - fMt24*tMf21))/100 - (981*fMb13*(fMt24*tMf23 - fMt34*tMf22))/100;
W[4][7]=-(981*tMb33)/100;
W[4][8]=0;
W[4][9]=(981*tMb13)/100;
W[4][10]=0;
W[5][0]=fMt24*tMf33 - fMt34*tMf32;
W[5][1]=fMt34*tMf31 - fMt14*tMf33;
W[5][2]=fMt14*tMf32 - fMt24*tMf31;
W[5][3]=tMf31;
W[5][4]=tMf32;
W[5][5]=tMf33;
W[5][6]=(981*fMb23*(fMt14*tMf33 - fMt34*tMf31))/100 - (981*fMb33*(fMt14*tMf32 - fMt24*tMf31))/100 - (981*fMb13*(fMt24*tMf33 - fMt34*tMf32))/100;
W[5][7]=(981*tMb23)/100;
W[5][8]=-(981*tMb13)/100;
W[5][9]=0;
W[5][10]=0;
	
	
	EstimatedForces=W*x;
	
	//printf("\n EstimatedForces=[%8.3f  %8.3f  %8.3f %8.3f  %8.3f %8.3f] \n",EstimatedForces[0],EstimatedForces[1],EstimatedForces[2],EstimatedForces[3],EstimatedForces[4],EstimatedForces[5]);
// Step 3.  Resolved= Measured-Estimated

	for(int i=0;i<6;i++) 
	{
		ResolvedForce2[i]=Ftcp_measured[i]-EstimatedForces[i]; // Assigning to a local variable
	}

	//printf("\n FUNC2: Resolved force  at TCP:=[%8.3f  %8.3f  %8.3f %8.3f  %8.3f %8.3f] \n",ResolvedForce2[0],ResolvedForce2[1],ResolvedForce2[2],ResolvedForce2[3],ResolvedForce2[4],ResolvedForce2[5]);

}



//*******************************************************************
// Discretizes the cutting path into many subpoints,
// This function also returns the "caps" of the cutting cylinder
//*******************************************************************

void CuttingTrajectory(double
 t,double tfinal,vpHomogeneousMatrix& oMp_t,vpColVector& CylinderCap1,vpColVector& CylinderCap2)
{
	// Declare all local variables
	vpHomogeneousMatrix Tinit,Tfinal;
	vpTranslationVector oTp_init,oTp_final; //Translation vector object to tool
	vpRotationMatrix oRp_init,oRp_final,Rotualpha,oRp_t;
	double r,theta;
	
	vpThetaUVector thetau;
	vpColVector u(3);
	

	Tinit[0][0]=1.0; Tinit[0][1]=0.0; Tinit[0][2]=0.0;   Tinit[0][3]=0.0;
	Tinit[1][0]=0.0 ; Tinit[1][1]=1.0; Tinit[1][2]=0.00;  Tinit[1][3]=0.0;
	Tinit[2][0]=0.0;  Tinit[2][1]=0.0;  Tinit[2][2]=1.0;   Tinit[2][3]=0.0;
	Tinit[3][0]=0.0 ; Tinit[3][1]=0.00; Tinit[3][2]=0.0; Tinit[3][3]=1.0;
	
	Tfinal[0][0]=1.0; Tfinal[0][1]=0.0; Tfinal[0][2]=0.0;   Tfinal[0][3]=-0.06;
	Tfinal[1][0]=0.0 ; Tfinal[1][1]=1.0; Tfinal[1][2]=0.00;  Tfinal[1][3]=0.18;
	Tfinal[2][0]=0.0;  Tfinal[2][1]=0.0;  Tfinal[2][2]=1.0;   Tfinal[2][3]=0.0;
	Tfinal[3][0]=0.0 ; Tfinal[3][1]=0.00; Tfinal[3][2]=0.0; Tfinal[3][3]=1.0;

	//
	Tinit.extract(oRp_init);//Rotation matrix R from HomogeneousMatrix
	Tinit.extract(oTp_init);//extract the deplacement vector
	Tfinal.extract(oRp_final);
	Tfinal.extract(oTp_final);


	// Extracting Cylinder caps
	CylinderCap1=oTp_init;
	CylinderCap2=oTp_final;
	if (t<tfinal)
	{
	// Calculating r
	r= 10*(pow(t/tfinal,3)) - 15*(pow(t/tfinal,4)) + 6*(pow(t/tfinal,5));
	// Calculating desired position and orientation


	Rotualpha=oRp_final*oRp_init;
	thetau.buildFrom(Rotualpha);
	theta = sqrt(pow(thetau[0],2)+pow(thetau[1],2)+pow(thetau[2],2));


	if(abs(theta)<0.001) // singular
	{
		for (int i=0;i<3;i++)
		{
			for (int j=0;j<3;j++)
			{
				Rotualpha[i][j]=0.0;
				if(i==j) 
				{
				Rotualpha[i][j]=1.0;
				}		
			}

		}
		
	}	
	else
	{
		u[0]=thetau[0]/theta;
		u[1]=thetau[1]/theta;
		u[2]=thetau[2]/theta;
		vpMatrix I(3,3);
		vpMatrix uskewsym(3,3);
		I.eye(3);
		SkewSym(u,uskewsym);
		theta=theta*r;
		Rotualpha = I-theta/2*uskewsym+(1-computesinc(theta)/pow(computesinc(theta/2),2))*uskewsym*uskewsym;
				
	}

	oRp_t=Rotualpha*oRp_init;

	for (int i=0;i<3;i++)
	{
		oMp_t[i][3]=oTp_init[i]+(r*(oTp_final[i]-oTp_init[i]));
	}

	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
		oMp_t[i][j]=oRp_t[i][j];
		}
	}
	}
 else
	{
 	oMp_t=Tfinal;
	}


}

//


double InCylinder(vpHomogeneousMatrix oMt,vpColVector CylinderCap1,vpColVector CylinderCap2,double Guideradius)
{

	vpColVector N(3),a(3),b(3),c(3),a1(3),oTt_vector(3);
	double vNorm,Test1,Test2,Distance;
	double Incylinder_Flag=0;
	vpTranslationVector oTt;
	vpThetaUVector thetau;
	oMt.extract(oTt);
	
	for(int i=0;i<3;i++)
	{
	oTt_vector[i]=oTt[i];
	}



	a=CylinderCap1-CylinderCap2; // axis of cylinder
	b=oTt_vector-CylinderCap2; // vector from cap 2 to point
	c=oTt_vector-CylinderCap1; // vector from cap 1 to point
	vNorm=VectorNorm(a,3);
	// Calculate the normal to the cylinder planes
	N[0]=(a[0])/vNorm;
	N[1]=(a[1])/vNorm;
	N[2]=(a[2])/vNorm;

	// Find distance from point to axis
  	a1 = vpColVector::crossProd(a,b); //The norm of this product gives the area of the parallelagram with sides a and b. Using the fact that area of a triangle is half this, the perpendicular distance from axis to point can be found.  
	Distance=VectorNorm(a1,3)/VectorNorm(a,3);
	printf("\n Distance to axis= %8.3f",Distance);

	Incylinder_Flag=Distance; // if outside radius but within cylinder, flag is distance

	if (Distance<Guideradius) // Cond. 1 within the cylinder raidus
		{
			Test1 = vpColVector::dotProd(b,N);
			Test2 = vpColVector::dotProd(c,N);
			if(Test1*Test2<=0) // Cond. 2 between planes of cylinder
			{
				Incylinder_Flag=1;
			}
		}
	


return Incylinder_Flag;
}


/************************************************************************************************

================================================================================================
================================================================================================

************************************************************************************************/



/************************************************************************************************


This set of functions are that will prove useful in later control laws, converted from equivalent Matlab ones, they have been already tested in Matla simulink

************************************************************************************************/

//Transformation matrix from SYMORO
void T70calc(vpColVector q, vpHomogeneousMatrix& T)
{

// DH PARAMETERS
double r1, r3, r5, r7;

r1=0.3105;
r3=0.4;
r5=0.39;
r7=0.078; // R7 can be changed to move TCP

q[1]=q[1]+1.57079633; // Compensating for difference between KRL and FRI


// q is the joint vector, this has been calculated by SYMORO
double U1T211, U1T212, U1T221, U1T222, U1T311, U1T312, U1T314, U1T321, U1T322, U1T324;
double U1T331, U1T332, U1T334, U1T411, U1T412, U1T421, U1T422, U1T431, U1T432, U1T511;
double U1T512, U1T514, U1T521, U1T522, U1T524, U1T531, U1T532, U1T534, U1T611, U1T612;
double U1T621, U1T622, U1T631, U1T632, U1T711, U1T712, U1T714, U1T721, U1T722, U1T724;
double U1T731, U1T732, U1T734;

	U1T211=cos(q[0])*sin(q[1]);
	U1T212=cos(q[0])*cos(q[1]);
	U1T221=sin(q[0])*sin(q[1]);
	U1T222=cos(q[1])*sin(q[0]);
	U1T311=U1T211*cos(q[2]) - sin(q[0])*sin(q[2]);
	U1T312=-(cos(q[2])*sin(q[0])) - U1T211*sin(q[2]);
	U1T314=r3*U1T212;
	U1T321=U1T221*cos(q[2]) + cos(q[0])*sin(q[2]);
	U1T322=cos(q[0])*cos(q[2]) - U1T221*sin(q[2]);
	U1T324=r3*U1T222;
	U1T331=-(cos(q[1])*cos(q[2]));
	U1T332=cos(q[1])*sin(q[2]);
	U1T334=r1 + r3*sin(q[1]);
	U1T411=U1T311*cos(q[3]) - U1T212*sin(q[3]);
	U1T412=-(U1T212*cos(q[3])) - U1T311*sin(q[3]);
	U1T421=U1T321*cos(q[3]) - U1T222*sin(q[3]);
	U1T422=-(U1T222*cos(q[3])) - U1T321*sin(q[3]);
	U1T431=U1T331*cos(q[3]) - sin(q[1])*sin(q[3]);
	U1T432=-(cos(q[3])*sin(q[1])) - U1T331*sin(q[3]);
	U1T511=U1T411*cos(q[4]) + U1T312*sin(q[4]);
	U1T512=U1T312*cos(q[4]) - U1T411*sin(q[4]);
	U1T514=U1T314 - r5*U1T412;
	U1T521=U1T421*cos(q[4]) + U1T322*sin(q[4]);
	U1T522=U1T322*cos(q[4]) - U1T421*sin(q[4]);
	U1T524=U1T324 - r5*U1T422;
	U1T531=U1T431*cos(q[4]) + U1T332*sin(q[4]);
	U1T532=U1T332*cos(q[4]) - U1T431*sin(q[4]);
	U1T534=U1T334 - r5*U1T432;
	U1T611=U1T511*cos(q[5]) - U1T412*sin(q[5]);
	U1T612=-(U1T412*cos(q[5])) - U1T511*sin(q[5]);
	U1T621=U1T521*cos(q[5]) - U1T422*sin(q[5]);
	U1T622=-(U1T422*cos(q[5])) - U1T521*sin(q[5]);
	U1T631=U1T531*cos(q[5]) - U1T432*sin(q[5]);
	U1T632=-(U1T432*cos(q[5])) - U1T531*sin(q[5]);
	U1T711=U1T611*cos(q[6]) + U1T512*sin(q[6]);
	U1T712=U1T512*cos(q[6]) - U1T611*sin(q[6]);
	U1T714=U1T514 + r7*U1T612;
	U1T721=U1T621*cos(q[6]) + U1T522*sin(q[6]);
	U1T722=U1T522*cos(q[6]) - U1T621*sin(q[6]);
	U1T724=U1T524 + r7*U1T622;
	U1T731=U1T631*cos(q[6]) + U1T532*sin(q[6]);
	U1T732=U1T532*cos(q[6]) - U1T631*sin(q[6]);
	U1T734=U1T534 + r7*U1T632;
	T[0][0]= U1T711;
	T[1][0]= U1T721;
	T[2][0]= U1T731;
	T[0][1]= U1T712;
	T[1][1]= U1T722;
	T[2][1]= U1T732;
	T[0][2]= U1T612;
	T[1][2]= U1T622;
	T[2][2]= U1T632;
	T[0][3]= U1T714;
	T[1][3]= U1T724;
	T[2][3]= U1T734;

}


// Jacobian matrix for SYMORO 
void J70calc(vpColVector q, vpMatrix& J)
{

// DH PARAMETERS
double r1, r3, r5, r7;

r1=0.3105;
r3=0.4;
r5=0.39;
r7=0.078; // R7 can be changed to move TCP

q[1]=q[1]+1.57079633; // Compensating for difference between KRL and FRI





J[0][0] = -(r3*cos(q[1])*sin(q[0])) - r5*cos(q[1])*cos(q[3])*sin(q[0])-r7*cos(q[1])*cos(q[3])*cos(q[5])*sin(q[0])-r5*cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])-r7*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3])-r5*cos(q[0])*sin(q[2])*sin(q[3]) - r7*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3])+r7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5])+r7*cos(q[0])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])-r7*cos(q[1])*cos(q[4])*sin(q[0])*sin(q[3])*sin(q[5])+r7*cos(q[0])*cos(q[2])*sin(q[4])*sin(q[5])-r7*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[1][0] = r3*cos(q[0])*cos(q[1]) + r5*cos(q[0])*cos(q[1])*cos(q[3])+r7*cos(q[0])*cos(q[1])*cos(q[3])*cos(q[5])+r5*cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])+r7*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])-r5*sin(q[0])*sin(q[2])*sin(q[3]) - r7*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[3])-r7*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5])+r7*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5])+r7*cos(q[0])*cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5])+r7*cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5])+r7*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[2][0] =0;

J[3][0] =0;

J[4][0] =0;

J[5][0] =1;

J[0][1] = -(r3*cos(q[0])*sin(q[1])) - r5*cos(q[0])*cos(q[3])*sin(q[1])-r7*cos(q[0])*cos(q[3])*cos(q[5])*sin(q[1])+r5*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3])+r7*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3])-r7*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5])-r7*cos(q[0])*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5])+r7*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[1][1] = -(r3*sin(q[0])*sin(q[1])) - r5*cos(q[3])*sin(q[0])*sin(q[1])-r7*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[1])+r5*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3])+r7*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[3])-r7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5])-r7*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5])+r7*cos(q[1])*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]);

J[2][1] = r3*cos(q[1]) + r5*cos(q[1])*cos(q[3]) + r7*cos(q[1])*cos(q[3])*cos(q[5])+r5*cos(q[2])*sin(q[1])*sin(q[3]) + r7*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])-r7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5])+r7*cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5])+r7*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[3][1] = sin(q[0]);

J[4][1] = -cos(q[0]);

J[5][1] =0;

J[0][2] = -(r5*cos(q[2])*sin(q[0])*sin(q[3])) - r7*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[3])-r5*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[3])-r7*cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])+r7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5])+r7*cos(q[0])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[5])+r7*cos(q[0])*cos(q[2])*sin(q[1])*sin(q[4])*sin(q[5])-r7*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]);

J[1][2] = r5*cos(q[0])*cos(q[2])*sin(q[3]) + r7*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[3])-r5*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])-r7*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])-r7*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5])+r7*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[5])+r7*cos(q[2])*sin(q[0])*sin(q[1])*sin(q[4])*sin(q[5])+r7*cos(q[0])*sin(q[2])*sin(q[4])*sin(q[5]);

J[2][2] = r5*cos(q[1])*sin(q[2])*sin(q[3]) + r7*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])-r7*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])-r7*cos(q[1])*cos(q[2])*sin(q[4])*sin(q[5]);

J[3][2] = cos(q[0])*cos(q[1]);

J[4][2] = cos(q[1])*sin(q[0]);

J[5][2] = sin(q[1]);

J[0][3] = r5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])+r7*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[5])*sin(q[1])-r5*cos(q[3])*sin(q[0])*sin(q[2]) - r7*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[2])-r5*cos(q[0])*cos(q[1])*sin(q[3]) - r7*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[3])+r7*cos(q[0])*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[5])+r7*cos(q[0])*cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5])-r7*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5]);

J[1][3] = r5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])+r7*cos(q[2])*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[1])+r5*cos(q[0])*cos(q[3])*sin(q[2]) + r7*cos(q[0])*cos(q[3])*cos(q[5])*sin(q[2])-r5*cos(q[1])*sin(q[0])*sin(q[3]) - r7*cos(q[1])*cos(q[5])*sin(q[0])*sin(q[3])+r7*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5])+r7*cos(q[2])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5])+r7*cos(q[0])*cos(q[4])*sin(q[2])*sin(q[3])*sin(q[5]);

J[2][3] = -(r5*cos(q[1])*cos(q[2])*cos(q[3])) - r7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[5])-r5*sin(q[1])*sin(q[3]) - r7*cos(q[5])*sin(q[1])*sin(q[3])+r7*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5])-r7*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[3])*sin(q[5]);

J[3][3] = -(cos(q[2])*sin(q[0])) - cos(q[0])*sin(q[1])*sin(q[2]);

J[4][3] = cos(q[0])*cos(q[2]) - sin(q[0])*sin(q[1])*sin(q[2]);

J[5][3] = cos(q[1])*sin(q[2]);

J[0][4] = r7*cos(q[2])*cos(q[4])*sin(q[0])*sin(q[5])+r7*cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[5])+r7*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])*sin(q[5])-r7*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5])-r7*cos(q[0])*cos(q[1])*sin(q[3])*sin(q[4])*sin(q[5]);

J[1][4] = -(r7*cos(q[0])*cos(q[2])*cos(q[4])*sin(q[5]))+r7*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[5])+r7*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])*sin(q[5])+r7*cos(q[0])*cos(q[3])*sin(q[2])*sin(q[4])*sin(q[5])-r7*cos(q[1])*sin(q[0])*sin(q[3])*sin(q[4])*sin(q[5]);

J[2][4] = -(r7*cos(q[1])*cos(q[4])*sin(q[2])*sin(q[5]))-r7*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4])*sin(q[5])-r7*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5]);

J[3][4] = cos(q[0])*cos(q[1])*cos(q[3]) + cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])-sin(q[0])*sin(q[2])*sin(q[3]);

J[4][4] = cos(q[1])*cos(q[3])*sin(q[0]) + cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])+cos(q[0])*sin(q[2])*sin(q[3]);

J[5][4] = cos(q[3])*sin(q[1]) - cos(q[1])*cos(q[2])*sin(q[3]);

J[0][5] = -(r7*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]))+r7*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[2])+r7*cos(q[0])*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[3])+r7*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[4])+r7*cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])-r7*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[5])-r7*cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])*sin(q[5])+r7*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5]);

J[1][5] = -(r7*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1]))-r7*cos(q[0])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2])+r7*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[3])-r7*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[4])+r7*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4])-r7*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[5])-r7*cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5])-r7*cos(q[0])*sin(q[2])*sin(q[3])*sin(q[5]);

J[2][5] = r7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])+r7*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3])-r7*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4]) - r7*cos(q[3])*sin(q[1])*sin(q[5])+r7*cos(q[1])*cos(q[2])*sin(q[3])*sin(q[5]);

J[3][5] = cos(q[2])*cos(q[4])*sin(q[0]) + cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])-cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - cos(q[0])*cos(q[1])*sin(q[3])*sin(q[4]);

J[4][5] = -(cos(q[0])*cos(q[2])*cos(q[4])) + cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])+cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])+cos(q[0])*cos(q[3])*sin(q[2])*sin(q[4]) - cos(q[1])*sin(q[0])*sin(q[3])*sin(q[4]);

J[5][5] = -(cos(q[1])*cos(q[4])*sin(q[2])) - cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4])-sin(q[1])*sin(q[3])*sin(q[4]);

J[0][6] =0;

J[1][6] =0;

J[2][6] =0;

J[3][6] = cos(q[0])*cos(q[1])*cos(q[3])*cos(q[5])+cos(q[0])*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])-cos(q[5])*sin(q[0])*sin(q[2])*sin(q[3])-cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5])+cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5])+cos(q[0])*cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5])+cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5])+cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[4][6] = cos(q[1])*cos(q[3])*cos(q[5])*sin(q[0])+cos(q[2])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3])+cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3])-cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5])-cos(q[0])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])+cos(q[1])*cos(q[4])*sin(q[0])*sin(q[3])*sin(q[5])-cos(q[0])*cos(q[2])*sin(q[4])*sin(q[5])+sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);

J[5][6] = cos(q[3])*cos(q[5])*sin(q[1]) - cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3])+cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5])+cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) - cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]);


}

//include: Px,Py,Pz,thetaUx,thetaUy,thetaUz,Fx,Fy,Fz,Omigax,Omigay,Omigaz,stiffness
void GETCICuttingPoints(char* filename, int num, double**& data)
{
  FILE *fp;  
  if((fp = fopen(filename, "rb"))==NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }
  char Contentl[250];//store one line
  char pBuf[20]; //store one number
  int hence;
  int counter;
  int k;
  hence = fseek(fp, 0, SEEK_SET);	//get to the beginning of file 
  fgets(Contentl, 250, fp);
   for(int i=0;i<num-1;i++)
  { 
    fgets(Contentl, 250, fp);
    counter = 0;
    while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    for(int j=0;j<24;j++)
    { 
      k=0;
      while(Contentl[counter] != ' '&& Contentl[counter] != '\n' )
      { 
        pBuf[k] = Contentl[counter];
        k++;
        counter++;
      }
      data[i][j] = atof(pBuf);
      memset(pBuf,'\0',sizeof pBuf);
      counter++;
      while(Contentl[counter] == ' ')
      { 
        counter++;
      }
    }
  }
}


//normalize the rotation matrix R
void NormalR(vpMatrix r , vpMatrix& R)
{
  vpColVector s(3);
  vpColVector n(3);
  vpColVector s1(3);
  vpColVector n1(3);
  vpColVector a1(3);
  double modules, modulen;

  for(int i=0;i<3;i++)
  {
    s[i] = r[i][0];
    n[i] = r[i][1];
  }
  modules=VectorNorm(s,3);
  modulen=VectorNorm(n,3);
  for(int i=0;i<3;i++)
  {
    s1[i] = s[i]/modules;
    n1[i] = n[i]/modulen;
  }
  a1 = vpColVector::crossProd(s1,n1);
  for(int i=0;i<3;i++)
  {
    R[i][0]=s1[i];
    R[i][1]=n1[i];
    R[i][2]=a1[i];
  }

}




/*
				FUNCTION: CHECK IF POINT IS IN POLYGON
//******************************************************************************************************************************************************
Arguments int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)

    nvert: Number of vertices in the polygon. Whether to repeat the first vertex at the end.
    vertx, verty: Arrays containing the x- and y-coordinates of the polygon's vertices.
    testx, testy: X- and y-coordinate of the test point.

It's both short and efficient and works both for convex and concave polygons. As suggested before, you should check the bounding rectangle first and treat polygon holes separately.

The idea behind this is pretty simple. It is based on the observation that a test point is within a polygon if when projected on the y-axis it's x value is below odd number of polygon edges. 

Copyright (c) 1970-2003, Wm. Randolph Franklin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimers.
    Redistributions in binary form must reproduce the above copyright notice in the documentation and/or other materials provided with the distribution.
    The name of W. Randolph Franklin may not be used to endorse or promote products derived from this Software without specific prior written permission. 

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

******************************************************************************************************************************************************

*/


int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

/*
				FUNCTION: CALUCLATE THE AREA OF A POLYGON
//******************************************************************************************************************************************************
Inputs: X and Y vertices given in a clockwise order, with points as the number of point
				
				
//  Public-domain function by Darel Rex Finley, 2006.

//*******************************************************************************************************************************************************
*/


double polygonArea(float *X, float *Y, int points) {

  double  area=0. ;
  int     i, j=points-1  ;

  for (i=0; i<points; i++) {
    area+=(X[j]+X[i])*(Y[j]-Y[i]); j=i; }

  return area*.5; }


 




/*
				FUNCTION: CALUCLATE THE DISTANCE BETWEEN 2D POINTS
//******************************************************************************************************************************************************
Inputs: x1, x2 ,y1,y2
				
Outputs: Distance between them
Function by Philip Long
//*******************************************************************************************************************************************************
*/ 
 double dist2D(double x1, double x2, double y1, double y2)
{
	double dx=(x2-x1);
	double dy=(y2-y1);
	double dist=sqrt(dx*dx+dy*dy);
    return dist;
}
  
  
/*
				FUNCTION: From the given blobs calculate the generic blobs using size and ellipical shape
//******************************************************************************************************************************************************
Inputs: blobs,filtering parameters
				
Outputs: Coorindates
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void filteringblobs(CBlobResult& blobs,int MaxArea, int MinArea, int AspectRatio)
{

	float ARthreshold; // this is the aspect ratio threshold the float version of param4
	// The output and temporary images


	
	//printf("number of original blobs= %d \n", blobs.GetNumBlobs());
	// Exclude blobs greater than certain area
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, MaxArea );
		
	//printf("number of blobs less than area= %d \n", blobs.GetNumBlobs());
	// Exlcude blobs less than certain area
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, MinArea  );
	//printf("number of blobs greater than= %d \n", blobs.GetNumBlobs());
	// Exclude blobs whose aspect ratio is greater than certain threshold
	// Eliminate all the blobs whose Aspect ratio (CBlobGetAxisRatio)is far from one
	// since AR=1 for circle when image plane is parallel to fork plane
	// we can reasonably assume that AR won't vary by massive amounnts
	// 1 is best 0 is worst, multiply by 100
	// =======================================================================// 
	ARthreshold=(float) AspectRatio;
	 blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
//    printf("number of blobs of required aspect ratio= %d \n", blobs.GetNumBlobs()); 	
	// display filtered blobs
 }
 
 
 /*
				FUNCTION: From the given blobs calculate the guide line 
//******************************************************************************************************************************************************
Inputs: blobs,filtering parameters
				
Outputs: Coorindates
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void filteringline(CBlobResult& blobs,int MaxArea, int MinArea, int length)
{
	CBlob BlobofInterest; // Blob used to find current information
	float ARthreshold; // this is the aspect ratio threshold the float version of param4
	// The output and temporary images
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	BlobofInterest=blobs.GetBlob(i);
	//printf("Area of blob of interest= %f \n",BlobofInterest.Area());
	}
	//printf("Before all exclusions %d \n",blobs.GetNumBlobs());
	//printf("number of original blobs= %d \n", blobs.GetNumBlobs());
	// Exclude blobs greater than certain area
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, MaxArea );
	//	printf("number after excluding maximum area area=%d \n",blobs.GetNumBlobs());
	//printf("number of blobs less than area= %d \n", blobs.GetNumBlobs());
	// Exlcude blobs less than certain area
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, MinArea  );
	//	printf("number after exculding mimium area min area=%d \n",blobs.GetNumBlobs());
	//printf("number of blobs greater than= %d \n", blobs.GetNumBlobs());

	// ARthreshold=(float) AspectRatio;
	 //blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	 ARthreshold=(float) length;
	 blobs.Filter( blobs, B_EXCLUDE,  CBlobGetLength(), B_LESS, ARthreshold);
    	
	// display filtered blobs
 }
 
 
 
 /*
				FUNCTION: From the given blobs calculate the forks blobs using apriori knowledge
//******************************************************************************************************************************************************
Inputs: blobs
				
Outputs: New set of Blobs
Function by Philip Long
//*******************************************************************************************************************************************************


*/


void forksblobs(CBlobResult blobs,int*  IndexofBlob)
{

	float Xij[2]; // The coorindates of the input blobs i  and j
    float Yij[2];
	double MinDiff;
	int f1,f2;
    f1=-1;f2=-1;
	CBlob Forkblob1,Forkblob2; // Blob used to find current information
	MinDiff=1000000; // Some very high number
	
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	 
	 Forkblob1=blobs.GetBlob(i);            
	 // Now find the blobs who correspond to fork
	 Xij[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
	 Yij[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
	
		for (int j=0;j<blobs.GetNumBlobs();j++)
		 {
			// If we have arrived here it means that a blob is nearly circular and within required area limits
			// therefore choose blobs who are closest to each other
			// in fact we should take the blobs who are closest to last known position
			
			
			Forkblob2=blobs.GetBlob(j); 
			if(i!=j)
			{
				 Xij[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
				 Yij[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
				 
				 
				 // Choosing blobs who are quite close and adding a bias for those towards the origin
				 if(( dist2D(Xij[0],Xij[1],Yij[0],Yij[1])+dist2D(Xij[0],0.0,Yij[0],0.0)) <MinDiff)
				 {
					 MinDiff=dist2D(Xij[0],Xij[1],Yij[0],Yij[1]);
	
					 f1=i;
					 f2=j;
                                      


					// printf("Index of blobs on the fork=%d,%d \n",f1,f2);
				 }
			}
		 }
	}
     IndexofBlob[0]=f1;
	 IndexofBlob[1]=f2;

		
 }
 
/*
				FUNCTION: From the given blobs calculate the forks blobs using apriori knowledge
        in this case we are looking for 4 blobs instead of two 
//******************************************************************************************************************************************************
Inputs: blobs
		
    Idea is to repeat the search bu exclude the blobs found on the last go		
Outputs: New set of Blobs
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void forks4blobs(CBlobResult blobs,int*  IndexofBlob)
{

	float Xij[2]; // The coorindates of the input blobs i  and j
    float Yij[2];
	double MinDiff;
	int f3,f4;
    f3=-1;f4=-1;
	CBlob Forkblob1,Forkblob2; // Blob used to find current information
	MinDiff=1000000; // Some very high number
	
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	 

	
		for (int j=0;j<blobs.GetNumBlobs();j++)
		 {
			// If we have arrived here it means that a blob is nearly circular and within required area limits
			// therefore choose blobs who are closest to each other
			// in fact we should take the blobs who are closest to last known position
			
			

			if(i!=j && i!=IndexofBlob[0] && i!=IndexofBlob[1] && j!=IndexofBlob[0] && j!=IndexofBlob[1])
			{
                Forkblob2=blobs.GetBlob(j); 
                Forkblob1=blobs.GetBlob(i);            
                // Now find the blobs who correspond to fork
                Xij[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
                Yij[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
				Xij[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
				Yij[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
				 
				 
				 // Chosing blobs who are close
				 if((dist2D(Xij[0],Xij[1],Yij[0],Yij[1]))<MinDiff)
				 {
					 MinDiff=dist2D(Xij[0],Xij[1],Yij[0],Yij[1]);
	
					 f3=i;
					 f4=j;
					// printf("Index of blobs on the fork=%d,%d \n",f1,f2);
				 }
			}
		 }
	}

     IndexofBlob[2]=f3;
     IndexofBlob[3]=f4;
		
 }
   /*
				FUNCTION: From the previous blobs try to track the using the previous positions
//******************************************************************************************************************************************************
Inputs: blobs, Old positions X and Y 
				
Outputs: New blobs whose centres are closest to previous blobs
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void forksblobstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob)
{

	float Xij[2]; // The coorindates of the input blobs i  and j
        float Yij[2];
	double MinDiff;
	int f1,f2;
	CBlob Forkblob1,Forkblob2; // Blob used to find current information
	MinDiff=1000000; // Some very high number
	f1=-1;
	f2=-1;

	
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	 
	 Forkblob1=blobs.GetBlob(i);            
	 // Now find the blobs who correspond to fork
	 Xij[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
	 Yij[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
	
		for (int j=0;j<blobs.GetNumBlobs();j++)
		 {
			// If we have arrived here it means that a blob is nearly circular and within required area limits
			// therefore choose blobs who are closest to each other
			// in fact we should take the blobs who are closest to last known position
			
			
			Forkblob2=blobs.GetBlob(j); 

			if(i!=j)
			{
				 Xij[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
				 Yij[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
				 
				 
				 // Current forks P1 P2, Old forks P3, P4 
				 // take min of [(P1,P3)+(P2,P4)] and [(P1,P4)+(P2,P3)] 
				
				 // Chosing blobs who are closest to last blobs
				 if( fmin( (dist2D(Xij[0],Xcentre[0],Yij[0],Ycentre[0])+  dist2D(Xij[1],Xcentre[1],Yij[1],Ycentre[1])),(dist2D(Xij[0],Xcentre[1],Yij[0],Ycentre[1])+  dist2D(Xij[1],Xcentre[0],Yij[1],Ycentre[0])))<MinDiff)
				 {
					 MinDiff= fmin( (dist2D(Xij[0],Xcentre[0],Yij[0],Ycentre[0])+  dist2D(Xij[1],Xcentre[1],Yij[1],Ycentre[1])),(dist2D(Xij[0],Xcentre[1],Yij[0],Ycentre[1])+  dist2D(Xij[1],Xcentre[0],Yij[1],Ycentre[0])));
	
					 f1=i;
					 f2=j;
				 }
			}
		 }
	}
     IndexofBlob[0]=f1;
	 IndexofBlob[1]=f2;

		
 }

 /*
				FUNCTION: From the previous blobs try to track the using the previous positions
//******************************************************************************************************************************************************
Inputs: blobs, Old positions X and Y 
				
Outputs: New blobs whose centres are closest to previous blobs
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void forks4blobstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob)
{

	float Xij[2]; // The coorindates of the input blobs i  and j
    float Yij[2];
	double MinDiff;
	int f3,f4;
	CBlob Forkblob1,Forkblob2; // Blob used to find current information
	MinDiff=1000000; // Some very high number
	f3=-1;
	f4=-1;

	
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	 
	 Forkblob1=blobs.GetBlob(i);            
	 // Now find the blobs who correspond to fork
	 Xij[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
	 Yij[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
	
		for (int j=0;j<blobs.GetNumBlobs();j++)
		 {
			// If we have arrived here it means that a blob is nearly circular and within required area limits
			// therefore choose blobs who are closest to each other
			// in fact we should take the blobs who are closest to last known position
			
			
			Forkblob2=blobs.GetBlob(j); 

            // Excluding the blobs that were found in the last track
			if(i!=j && i!=IndexofBlob[0] && i!=IndexofBlob[1] && j!=IndexofBlob[0] && j!=IndexofBlob[1])
			{
				 Xij[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
				 Yij[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
				 
				 
				 // Current forks P1 P2, Old forks P3, P4 
				 // take min of [(P1,P3)+(P2,P4)] and [(P1,P4)+(P2,P3)] 

				 // Chosing blobs who are closest to last blobs
				 if( fmin( (dist2D(Xij[0],Xcentre[2],Yij[0],Ycentre[2])+  dist2D(Xij[1],Xcentre[3],Yij[1],Ycentre[3])),(dist2D(Xij[0],Xcentre[3],Yij[0],Ycentre[3])+  dist2D(Xij[1],Xcentre[2],Yij[1],Ycentre[2])))<MinDiff)
				 {
					 MinDiff= fmin( (dist2D(Xij[0],Xcentre[2],Yij[0],Ycentre[2])+  dist2D(Xij[1],Xcentre[3],Yij[1],Ycentre[3])),(dist2D(Xij[0],Xcentre[3],Yij[0],Ycentre[3])+  dist2D(Xij[1],Xcentre[2],Yij[1],Ycentre[2])));
	
					 f3=i;
					 f4=j;
					// printf("Index of blobs on the fork=%d,%d \n",f1,f2);
				 }
			}
		 }
	}
     IndexofBlob[2]=f3;
	 IndexofBlob[3]=f4;

		
 }


 /*
				FUNCTION:OrderForkBlobs reaarnage the obtained blobs into a standard order, closest to origin
//******************************************************************************************************************************************************
Inputs: blobs, Indexofblobs
				
Outputs: Indexofblobs so that blob[0] is closest to origin and blob[4] is furthest away
Function by Philip Long
//*******************************************************************************************************************************************************


*/

  void OrderForkBlobs(CBlobResult blobs,int* IndexofBlob)
{

    CBlob Forkblob1,Forkblob2,Forkblob3,Forkblob4; // Blob used to find current information

    vpColVector Distance(4);


    double X1,X2,X3,X4,Y1,Y2,Y3,Y4;
  
    if(IndexofBlob[0]!=-1 && IndexofBlob[1]!=-1 && IndexofBlob[2]!=-1 && IndexofBlob[3]!=-1)
    {       
        Forkblob1=blobs.GetBlob(IndexofBlob[0]);Forkblob2=blobs.GetBlob(IndexofBlob[1]);
        Forkblob3=blobs.GetBlob(IndexofBlob[2]);Forkblob4=blobs.GetBlob(IndexofBlob[3]);
        // get centre points of the blobs


        X1=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
        X2=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
        X3=Forkblob3.MinX() + (( Forkblob3.MaxX() - Forkblob3.MinX() ) / 2.0);
        X4=Forkblob4.MinX() + (( Forkblob4.MaxX() - Forkblob4.MinX() ) / 2.0);
        Y1=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
        Y2=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
        Y3=Forkblob3.MinY() + (( Forkblob3.MaxY() - Forkblob3.MinY() ) / 2.0);
        Y4=Forkblob4.MinY() + (( Forkblob4.MaxY() - Forkblob4.MinY() ) / 2.0);
     
        Distance[0]=dist2D(X1,0,Y1,0);    Distance[1]=dist2D(X2,0,Y2,0);    Distance[2]=dist2D(X3,0,Y3,0);    Distance[3]=dist2D(X4,0,Y4,0);
        
       

        vpColVector SortedDistances = vpColVector::sort(Distance);  
       for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
              if(Distance[j]==SortedDistances[i])
                {
                IndexofBlob[i]=j;
                }
            }
        }
    }

  //    printf("SortedDistances=%f,%f,%f,%f",SortedDistances[0],SortedDistances[1],SortedDistances[2],SortedDistances[3]);
  //    printf("Index=%f,%f,%f,%f",IndexofBlob[0],IndexofBlob[1],IndexofBlob[2],IndexofBlob[3]);
  //    cvWaitKey(1000);
}

 void fitsdotstoBlob(CBlobResult blobs,int* IndexofBlob,float* Xcentre,float* Ycentre)
{

    CBlob Forkblob1,Forkblob2; // Blob used to find current information

    Forkblob1=blobs.GetBlob(IndexofBlob[0]);Forkblob2=blobs.GetBlob(IndexofBlob[1]);

    // Update last known location
    Xcentre[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
    Xcentre[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
    Ycentre[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
    Ycentre[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);

}

 void fits4dotstoBlob(CBlobResult blobs,int* IndexofBlob,float* Xcentre,float* Ycentre)
{

    CBlob Forkblob1,Forkblob2,Forkblob3,Forkblob4; // Blob used to find current information

    Forkblob1=blobs.GetBlob(IndexofBlob[0]);Forkblob2=blobs.GetBlob(IndexofBlob[1]);
    Forkblob3=blobs.GetBlob(IndexofBlob[2]);Forkblob4=blobs.GetBlob(IndexofBlob[3]);

    // Update last known location
    Xcentre[0]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0);
    Xcentre[1]=Forkblob2.MinX() + (( Forkblob2.MaxX() - Forkblob2.MinX() ) / 2.0);
    Xcentre[2]=Forkblob3.MinX() + (( Forkblob3.MaxX() - Forkblob3.MinX() ) / 2.0);
    Xcentre[3]=Forkblob4.MinX() + (( Forkblob4.MaxX() - Forkblob4.MinX() ) / 2.0);
    Ycentre[0]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);
    Ycentre[1]=Forkblob2.MinY() + (( Forkblob2.MaxY() - Forkblob2.MinY() ) / 2.0);
    Ycentre[2]=Forkblob3.MinY() + (( Forkblob3.MaxY() - Forkblob3.MinY() ) / 2.0);
    Ycentre[3]=Forkblob4.MinY() + (( Forkblob4.MaxY() - Forkblob4.MinY() ) / 2.0);

}


  /*
				FUNCTION: From the given blobs calculate the best line using apriori knowledge
//******************************************************************************************************************************************************
Inputs: blobs, 
				
Outputs: IndexofBlob of LineBlob
Function by Philip Long
//*******************************************************************************************************************************************************

*/
 void Lineblob(CBlobResult blobs,int* IndexofBlob)
{

	double MaxDiff;
	int f1,f2;
	CBlob LineBlob1; // Blob used to find current information
	MaxDiff=0; // Some very high number
	f1=-1;
	f2=-1;

	CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
	CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
	CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
	CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 
	CBlobGetMinX minX = CBlobGetMinX();
	CBlobGetMaxX maxX = CBlobGetMaxX();
	CBlobGetMinY minY = CBlobGetMinY();
	CBlobGetMaxY maxY = CBlobGetMaxY();

	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	 
	 LineBlob1=blobs.GetBlob(i);            
	 // Now find the blobs who correspond to fork
		float x1=maxxmaxy(LineBlob1);
		float x2=minxminy(LineBlob1);
		float y1=maxY(LineBlob1);
		float y2=minY(LineBlob1);
		 
		// The line should pan the screen therefore the distance between the corner should be the longest possible
		if((dist2D(x1,x2,y1,y2))>MaxDiff)
		{
		 MaxDiff=dist2D(x1,x2,y1,y2);
		 f1=i;
		 f2=10;
		}
			
		 
	}
     IndexofBlob[0]=f1;
	 IndexofBlob[1]=f2;

		
 }
 
 

 



  /*
				FUNCTION: From the previous blobs try to track the using the previous positions
//******************************************************************************************************************************************************
Inputs: blobs, Old positions X and Y 
				
Outputs: New Line whose centres are closest to previous blobs
Function by Philip Long
//*******************************************************************************************************************************************************


*/

void Linetrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int* IndexofBlob)
{

float Xij[2]; // The coorindates of the input blobs i  and j
float Yij[2];
double MinDiff;
int f1,f2;
CBlob LineBlob1; // Blob used to find current information
MinDiff=1000000; // Some very high number
f1=-1;
f2=-1;

CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 
CBlobGetMinX minX = CBlobGetMinX();
CBlobGetMaxX maxX = CBlobGetMaxX();
CBlobGetMinY minY = CBlobGetMinY();
CBlobGetMaxY maxY = CBlobGetMaxY();

for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{

	// Now find the blobs who correspond to fork
	LineBlob1=blobs.GetBlob(i);            
	// Now find the blobs who correspond to fork
	float x1=maxxmaxy(LineBlob1);
	float x2=minxminy(LineBlob1);
	float y1=maxY(LineBlob1);
	float y2=minY(LineBlob1);


	// Current forks P1 P2, Old forks P3, P4 
	// take min of [(P1,P3)+(P2,P4)] and [(P1,P4)+(P2,P3)] 

	// Chosing blobs who are closest to last blobs
	if( fmin( (dist2D(x1,Xcentre[0],y1,Ycentre[0])+  dist2D(x2,Xcentre[1],y2,Ycentre[1])),(dist2D(x1,Xcentre[1],y1,Ycentre[1])+  dist2D(x2,Xcentre[0],y2,Ycentre[0])))<MinDiff)
		{
		MinDiff= fmin( (dist2D(x1,Xcentre[0],y1,Ycentre[0])+  dist2D(x2,Xcentre[1],y2,Ycentre[1])),(dist2D(x1,Xcentre[1],y1,Ycentre[1])+  dist2D(x2,Xcentre[0],y2,Ycentre[0])));

		f1=i;
		f2=10;
		}

	}
	
IndexofBlob[0]=f1;
IndexofBlob[1]=f2;


}

  /*
				FUNCTION: A small function to check if index is good
//******************************************************************************************************************************************************
Inputs: index, ensuring that the index is not equal to -1! and also checks that the IndexofBlob
has no repeated elements

Outputs a int 0 1

				
//*******************************************************************************************************************************************************
*/
int ValidityIndexofBlob(int*  IndexofBlob)
{
int Flag=1;// 1 is ok
    for(int i=0;i<4;i++)    
    {
        for (int j=0;j<4;j++)
        {
            if(i!=j)
            {
                if(IndexofBlob[i]==IndexofBlob[j] || IndexofBlob[i]==-1)
                {
                    Flag=0;
                }
            }
        }   
    }
return Flag;
    
}


  /*
				FUNCTION: From the previous blobs try to track the using the previous positions

//******************************************************************************************************************************************************
Inputs: blobs, Old positions X and Y 
				
Outputs: New blobs whose centres are closest to previous blobs
Function by Philip Long

//*******************************************************************************************************************************************************


*/
void forksblobAllstrack(CBlobResult blobs,float* Xcentre,float* Ycentre,int*  IndexofBlob)
{

	float Xij[2]; // The coorindates of the input blobs i  and j
    float Yij[2];
	double MinDiff;
	CBlob Forkblob1,Forkblob2; // Blob used to find current information
	

    for (int j=0;j<4;j++)
    {
        MinDiff=1000000; // Some very high number
        Xij[0]=Xcentre[j]; // Old centres of tracked blobs
        Yij[0]=Ycentre[j]; // Old centres of tracked blobs
        
        for (int i = 0; i < blobs.GetNumBlobs(); i++ ) // Compare with all new found blobs
        {

            Forkblob1=blobs.GetBlob(i);            
            Xij[1]=Forkblob1.MinX() + (( Forkblob1.MaxX() - Forkblob1.MinX() ) / 2.0); // Centre of blobs i
            Yij[1]=Forkblob1.MinY() + (( Forkblob1.MaxY() - Forkblob1.MinY() ) / 2.0);

            // Chosing blobs who are closest to last blobs
            if( (dist2D(Xij[0],Xij[1],Yij[0],Yij[1])<MinDiff))
            {
                 MinDiff= dist2D(Xij[0],Xij[1],Yij[0],Yij[1]);
                 IndexofBlob[j]=i;
            // printf("Index of blobs on the fork=%d,%d \n",f1,f2);
            }
        }
    }	
 }

  /*
				FUNCTION: From a known blob fit the longest line
//******************************************************************************************************************************************************
Inputs: blob
				
Outputs: Xcentre,Ycentre the parameters of the segment to be servoed

Summary: The camera views a blob and locates the minimum point ymin and xmin at this value. 
This will correspond to the bottom of the screen. The float thickness is added to the xvalue to centre it a bit
Next from the orginal point lines are draw in several directions ( 0:pi/12:pi) until they exit the blob. The direction
that results in the longest line is save this this line becomes the segment that we would like to servo.

Example if the line is perfectly vertical the line at angle pi/2 spans the entire screen and we can servo
it.



Function by Philip Long
//*******************************************************************************************************************************************************


*/

void fitLinetoBlob(CBlob blobtest,float* Xcentre,float* Ycentre)
{

	CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
	CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
	CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
	CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 
	CBlobGetMinX minX = CBlobGetMinX();
	CBlobGetMaxX maxX = CBlobGetMaxX();
	CBlobGetMinY minY = CBlobGetMinY();
	CBlobGetMaxY maxY = CBlobGetMaxY();
	
	float thickness=10;// approxiamate thickness of the line
	float AngleIncrement=PI/20;
	float x1=maxxmaxy(blobtest)-thickness;
	float y1=maxY(blobtest);
	float x2,y2;

	CvPoint2D32f pt3;

	float Maxincreament=0;

	// The idea is I fan out in many directions, the furtherst direction I can reach while still in the 
	// blob is chosen as the guide line direction


	for(float i=0;i<(PI);i+=AngleIncrement)
	{

		float SearchRange=5;
		int j=1; 

		while(j==1)
		{ 

			pt3.x=x1+SearchRange*cos(i);
			pt3.y=y1-SearchRange*sin(i);

			if(SearchRange>Maxincreament)
			{
				Maxincreament=SearchRange;
				x2=pt3.x;
				y2=pt3.y;
			}


			CBlobGetXYInside insidePoint=CBlobGetXYInside(pt3);
			j=insidePoint(blobtest);
			SearchRange=SearchRange+10.0; // increase by 10 pixels

		}
	}		

	Xcentre[0]=x1;	Xcentre[1]=x2;Ycentre[0]=y1;Ycentre[1]=y2;
}


  /*
				FUNCTION: Compute Interaction matrix for segment
//******************************************************************************************************************************************************
Inputs: The image position of the blobs: Xcentre={u1,u2} and Ycentre={v1,v2} and the depth DepthZ. Camera parameters (fu,fv,fs,u0,v0)

The camera parameters are from collineation matrix of pinhole camera such that:

_______________________________________

[u]    [ fu,fs,u0]   [Xcentre/DepthZ] 
[v]  = [ 0 ,fv,v0] * [Ycentre/DepthZ] 
[1]    [ 0, 0 ,1]		[1]	          
_______________________________________


Outputs: Interaction matrix relating image segment velocity to camera Cartesian velocity Lseg

Function by Philip Long
//*******************************************************************************************************************************************************

*/


void InteractionMatSegment(vpMatrix& Lseg,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters)
{
float uc,vc,l,theta,fu,fv,fs,u0,v0,Z1,Z2; // Parameters to describe dots as segment


fu=CameraParameters[0];
fv=CameraParameters[1];
fs=CameraParameters[2];
u0=CameraParameters[3];
v0=CameraParameters[4];

Z1=DepthZ[0];
Z2=DepthZ[1];


uc=(Xcentre[0]+Xcentre[1])/2;
vc=(Ycentre[0]+Ycentre[1])/2;
l=pow( (pow((Xcentre[0]-Xcentre[1]),2) + pow((Ycentre[0]-Ycentre[1]),2) ), 0.5);
theta=atan((Ycentre[0]-Ycentre[1])/(Xcentre[0]-Xcentre[1]));    

/*
		 printf(" xp = %f \n",uc);
		 printf(" yp = %f \n",vc);
		 printf(" Lp = %f \n",l);
		 printf(" theta = %f \n",theta);
*/
Lseg[0][0]=- fu/(2*Z1) - fu/(2*Z2);
Lseg[0][1]=- fs/(2*Z1) - fs/(2*Z2);
Lseg[0][2]=(uc - u0 + (l*cos(theta))/2)/(2*Z1) - (u0 - uc + (l*cos(theta))/2)/(2*Z2);
Lseg[0][3]=fs + (u0*v0 - u0*vc - uc*v0 + uc*vc + (pow(l,2)*sin(2*theta))/8)/fv;
Lseg[0][4]=(fs*u0*v0 - fs*u0*vc - fs*uc*v0 + fs*uc*vc + (fs*pow(l,2)*sin(2*theta))/8)/(fu*fv) - ((pow(l,2)*pow(cos(theta),2))/4 - 2*u0*uc + pow(fu,2) + pow(u0,2) + pow(uc,2))/fu;
Lseg[0][5]=(fs*(u0 - uc))/fu - ((v0 - vc)*(pow(fs,2) + pow(fu,2)))/(fu*fv);
Lseg[1][0]=0;
Lseg[1][1]=- fv/(2*Z1) - fv/(2*Z2);
Lseg[1][2]=(vc - v0 + (l*sin(theta))/2)/(2*Z1) - (v0 - vc + (l*sin(theta))/2)/(2*Z2);
Lseg[1][3]=fv + ((pow(l,2)*pow(sin(theta),2))/4 - 2*v0*vc + pow(v0,2) + pow(vc,2))/fv;
Lseg[1][4]=(fs*pow(v0,2) + fs*pow(vc,2) - 2*fs*v0*vc + (fs*pow(l,2)*pow(sin(theta),2))/4)/(fu*fv) - (u0*v0 - u0*vc - uc*v0 + uc*vc + (pow(l,2)*sin(2*theta))/8)/fu;
Lseg[1][5]=(fv*u0 - fv*uc - fs*v0 + fs*vc)/fu;
Lseg[2][0]=(fu*l*cos(theta)*(Z1 - Z2))/(Z1*Z2*l);
Lseg[2][1]=(l*(Z1 - Z2)*(fs*cos(theta) + fv*sin(theta)))/(Z1*Z2*l);
Lseg[2][2]=(l*(l - 2*u0*cos(theta) + 2*uc*cos(theta) - 2*v0*sin(theta) + 2*vc*sin(theta)))/(2*Z1*l) + (l*(Z1*l + 2*Z1*u0*cos(theta) - 2*Z1*uc*cos(theta) + 2*Z1*v0*sin(theta) - 2*Z1*vc*sin(theta)))/(2*Z1*Z2*l);
Lseg[2][3]=-(2*v0*l - 2*vc*l + u0*sin(2*theta)*l - uc*sin(2*theta)*l + 2*v0*pow(sin(theta),2)*l - 2*vc*pow(sin(theta),2)*l)/(2*fv);
Lseg[2][4]=(l*(3*u0 - 3*uc + u0*cos(2*theta) - uc*cos(2*theta) + v0*sin(2*theta) - vc*sin(2*theta)))/(2*fu) - (l*(3*fs*v0 - 3*fs*vc - fs*v0*cos(2*theta) + fs*vc*cos(2*theta) + fs*u0*sin(2*theta) - fs*uc*sin(2*theta)))/(2*fu*fv);
Lseg[2][5]=(l*(pow(fs,2)*sin(2*theta) + pow(fu,2)*sin(2*theta) - pow(fv,2)*sin(2*theta) - 2*fs*fv*cos(2*theta)))/(2*fu*fv);
Lseg[3][0]=-(fu*sin(theta)*(Z1 - Z2))/(Z1*Z2*l);
Lseg[3][1]=((Z1 - Z2)*(fv*cos(theta) - fs*sin(theta)))/(Z1*Z2*l);
Lseg[3][2]=((Z1 - Z2)*(v0*cos(theta) - vc*cos(theta) - u0*sin(theta) + uc*sin(theta)))/(Z1*Z2*l);
Lseg[3][3]=(u0*pow(sin(theta),2) - uc*pow(sin(theta),2) - (v0*sin(2*theta))/2 + (vc*sin(2*theta))/2)/fv;
Lseg[3][4]=(v0 - vc - (u0*sin(2*theta))/2 + (uc*sin(2*theta))/2 - v0*pow(sin(theta),2) + vc*pow(sin(theta),2))/fu + (fs*u0*pow(sin(theta),2) - fs*uc*pow(sin(theta),2) - (fs*v0*sin(2*theta))/2 + (fs*vc*sin(2*theta))/2)/(fu*fv);
Lseg[3][5]=(fv*(pow(sin(theta),2) - 1))/fu - (pow(fs,2)*pow(sin(theta),2) + pow(fu,2)*pow(sin(theta),2))/(fu*fv) + (fs*sin(2*theta))/fu;

}

  /*
				FUNCTION: Compute Interaction matrix for 4 points
//******************************************************************************************************************************************************
Inputs: The image position of the blobs: Xcentre={u1,u2,u3,u4} and Ycentre={v1,v2,v3,v4} and the depth DepthZ. Camera parameters (fu,fv,fs,u0,v0)


The camera parameters are from collineation matrix of pinhole camera such that:

_______________________________________

[u]    [ fu,fs,u0]   [Xcentre/DepthZ] 

[v]  = [ 0 ,fv,v0] * [Ycentre/DepthZ] 
[1]    [ 0, 0 ,1]		[1]	          
_______________________________________


Outputs: Interaction matrix relating image point velocity to camera Cartesian velocity Lseg

s=[x1,y1,x2,y2,x3,y3,x4,y4]

Function by Philip Long
//*******************************************************************************************************************************************************

*/

 /*
				FUNCTION: Compute Interaction matrix for point
//******************************************************************************************************************************************************
Inputs: The image position of the blob: {u1} and {v1} and the depth DepthZ.
 Camera parameters (fu,fv,fs,u0,v0)


The camera parameters are from collineation matrix of pinhole camera such that:

_______________________________________

[u]    [ fu,fs,u0]   [Xcentre/DepthZ] 

[v]  = [ 0 ,fv,v0] * [Ycentre/DepthZ] 
[1]    [ 0, 0 ,1]		[1]	          
_______________________________________


Outputs: Interaction matrix relating image segment velocity to camera Cartesian velocity Lseg


Function by Philip Long
//*******************************************************************************************************************************************************

*/

void InteractionMatOnePoint(vpMatrix& Lpts,float u1,float v1,float DepthZ,float* CameraParameters)
{
float u0,v0,fu,fv,fs,Z1; // Parameters to describe dots as image points



fu=CameraParameters[0];
fv=CameraParameters[1];
fs=CameraParameters[2];
u0=CameraParameters[3];
v0=CameraParameters[4];

Z1=DepthZ;


Lpts[0][0]=-fu/Z1;
Lpts[0][1]=-fs/Z1;
Lpts[0][2]=-(u0 - u1)/Z1;
Lpts[0][3]=fs + ((u0 - u1)*(v0 - v1))/fv;
Lpts[0][4]=- fu - ((u0 - u1)*(fv*u0 - fv*u1 - fs*v0 + fs*v1))/(fu*fv);
Lpts[0][5]=(fs*(u0 - u1))/fu - ((v0 - v1)*(pow(fs,2) + pow(fu,2)))/(fu*fv);
Lpts[1][0]=0;
Lpts[1][1]=-fv/Z1;
Lpts[1][2]=-(v0 - v1)/Z1;
Lpts[1][3]=fv + pow((v0 - v1),2)/fv;
Lpts[1][4]=(fs*pow((v0 - v1),2))/(fu*fv) - ((u0 - u1)*(v0 - v1))/fu;
Lpts[1][5]=(fv*(u0 - u1))/fu - (fs*(v0 - v1))/fu;
}


  /*
				FUNCTION: Compute Interaction matrix for 4 points
//******************************************************************************************************************************************************
Inputs: The image position of the blobs: Xcentre={u1,u2,u3,u4} and Ycentre={v1,v2,v3,v4} and the depth DepthZ. Camera parameters (fu,fv,fs,u0,v0)


The camera parameters are from collineation matrix of pinhole camera such that:

_______________________________________

[u]    [ fu,fs,u0]   [Xcentre/DepthZ] 

[v]  = [ 0 ,fv,v0] * [Ycentre/DepthZ] 
[1]    [ 0, 0 ,1]		[1]	          
_______________________________________


Outputs: Interaction matrix relating image point velocity to camera Cartesian velocity Lseg

s=[x1,y1,x2,y2,x3,y3,x4,y4]

Function by Philip Long
//*******************************************************************************************************************************************************

*/

void InteractionMatFourPoints(vpMatrix& L4pts,vpMatrix Lpts1,vpMatrix Lpts2)
{
    // The interaction matrix for 4 points is equal to the interaction of 2 points
    // stacked
    for(int i=0;i<8;i++)
    {
        for (int j=0;j<6;j++)
        {
        if(i<4)
        {
        L4pts[i][j]=Lpts1[i][j];
        }
        else
        {
        L4pts[i][j]=Lpts2[i-4][j];
        }
        }
    }

}

void InteractionMatSegmentReduced(vpMatrix& Lseg_r,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters)
{
float uc,vc,l,theta,fu,fv,fs,u0,v0,Z1,Z2; // Parameters to describe dots as segment


fu=CameraParameters[0];
fv=CameraParameters[1];
fs=CameraParameters[2];
u0=CameraParameters[3];
v0=CameraParameters[4];

Z1=DepthZ[0];
Z2=DepthZ[1];


uc=(Xcentre[0]+Xcentre[1])/2;
vc=(Ycentre[0]+Ycentre[1])/2;
l=pow( (pow((Xcentre[0]-Xcentre[1]),2) + pow((Ycentre[0]-Ycentre[1]),2) ), 0.5);
theta=atan((Ycentre[0]-Ycentre[1])/(Xcentre[0]-Xcentre[1]));    


/*
		 printf(" xp = %f \n",uc);
		 printf(" yp = %f \n",vc);
		 printf(" Lp = %f \n",l);
		 printf(" theta = %f \n",theta);
*/
Lseg_r[0][0]=- fu/(2*Z1) - fu/(2*Z2);
Lseg_r[0][1]=- fs/(2*Z1) - fs/(2*Z2);
Lseg_r[0][2]=(uc - u0 + (l*cos(theta))/2)/(2*Z1) - (u0 - uc + (l*cos(theta))/2)/(2*Z2);
Lseg_r[0][3]=fs + (u0*v0 - u0*vc - uc*v0 + uc*vc + (pow(l,2)*sin(2*theta))/8)/fv;
Lseg_r[0][4]=(fs*u0*v0 - fs*u0*vc - fs*uc*v0 + fs*uc*vc + (fs*pow(l,2)*sin(2*theta))/8)/(fu*fv) - ((pow(l,2)*pow(cos(theta),2))/4 - 2*u0*uc + pow(fu,2) + pow(u0,2) + pow(uc,2))/fu;
Lseg_r[0][5]=(fs*(u0 - uc))/fu - ((v0 - vc)*(pow(fs,2) + pow(fu,2)))/(fu*fv);
Lseg_r[1][0]=0;
Lseg_r[1][1]=- fv/(2*Z1) - fv/(2*Z2);
Lseg_r[1][2]=(vc - v0 + (l*sin(theta))/2)/(2*Z1) - (v0 - vc + (l*sin(theta))/2)/(2*Z2);
Lseg_r[1][3]=fv + ((pow(l,2)*pow(sin(theta),2))/4 - 2*v0*vc + pow(v0,2) + pow(vc,2))/fv;
Lseg_r[1][4]=(fs*pow(v0,2) + fs*pow(vc,2) - 2*fs*v0*vc + (fs*pow(l,2)*pow(sin(theta),2))/4)/(fu*fv) - (u0*v0 - u0*vc - uc*v0 + uc*vc + (pow(l,2)*sin(2*theta))/8)/fu;
Lseg_r[1][5]=(fv*u0 - fv*uc - fs*v0 + fs*vc)/fu;
/*
Lseg_r[2][0]=(fu*l*cos(theta)*(Z1 - Z2))/(Z1*Z2*l);
Lseg_r[2][1]=(l*(Z1 - Z2)*(fs*cos(theta) + fv*sin(theta)))/(Z1*Z2*l);
Lseg_r[2][2]=(l*(l - 2*u0*cos(theta) + 2*uc*cos(theta) - 2*v0*sin(theta) + 2*vc*sin(theta)))/(2*Z1*l) + (l*(Z1*l + 2*Z1*u0*cos(theta) - 2*Z1*uc*cos(theta) + 2*Z1*v0*sin(theta) - 2*Z1*vc*sin(theta)))/(2*Z1*Z2*l);
Lseg_r[2][3]=-(2*v0*l - 2*vc*l + u0*sin(2*theta)*l - uc*sin(2*theta)*l + 2*v0*pow(sin(theta),2)*l - 2*vc*pow(sin(theta),2)*l)/(2*fv);
Lseg_r[2][4]=(l*(3*u0 - 3*uc + u0*cos(2*theta) - uc*cos(2*theta) + v0*sin(2*theta) - vc*sin(2*theta)))/(2*fu) - (l*(3*fs*v0 - 3*fs*vc - fs*v0*cos(2*theta) + fs*vc*cos(2*theta) + fs*u0*sin(2*theta) - fs*uc*sin(2*theta)))/(2*fu*fv);
Lseg_r[2][5]=(l*(pow(fs,2)*sin(2*theta) + pow(fu,2)*sin(2*theta) - pow(fv,2)*sin(2*theta) - 2*fs*fv*cos(2*theta)))/(2*fu*fv);*/
Lseg_r[2][0]=-(fu*sin(theta)*(Z1 - Z2))/(Z1*Z2*l);
Lseg_r[2][1]=((Z1 - Z2)*(fv*cos(theta) - fs*sin(theta)))/(Z1*Z2*l);
Lseg_r[2][2]=((Z1 - Z2)*(v0*cos(theta) - vc*cos(theta) - u0*sin(theta) + uc*sin(theta)))/(Z1*Z2*l);
Lseg_r[2][3]=(u0*pow(sin(theta),2) - uc*pow(sin(theta),2) - (v0*sin(2*theta))/2 + (vc*sin(2*theta))/2)/fv;
Lseg_r[2][4]=(v0 - vc - (u0*sin(2*theta))/2 + (uc*sin(2*theta))/2 - v0*pow(sin(theta),2) + vc*pow(sin(theta),2))/fu + (fs*u0*pow(sin(theta),2) - fs*uc*pow(sin(theta),2) - (fs*v0*sin(2*theta))/2 + (fs*vc*sin(2*theta))/2)/(fu*fv);
Lseg_r[2][5]=(fv*(pow(sin(theta),2) - 1))/fu - (pow(fs,2)*pow(sin(theta),2) + pow(fu,2)*pow(sin(theta),2))/(fu*fv) + (fs*sin(2*theta))/fu;

}
  /*
				FUNCTION: Compute Interaction matrix for points
//******************************************************************************************************************************************************
Inputs: The image position of the blobs: Xcentre={u1,u2} and Ycentre={v1,v2} and the depth DepthZ. Camera parameters (fu,fv,fs,u0,v0)

The camera parameters are from collineation matrix of pinhole camera such that:

_______________________________________

[u]    [ fu,fs,u0]   [Xcentre/DepthZ] 
[v]  = [ 0 ,fv,v0] * [Ycentre/DepthZ] 
[1]    [ 0, 0 ,1]		[1]	          
_______________________________________


Outputs: Interaction matrix relating image segment velocity to camera Cartesian velocity Lseg

Function by Philip Long
//*******************************************************************************************************************************************************

*/

void InteractionMatTwoPoints(vpMatrix& Lpts,float* Xcentre,float* Ycentre,float* DepthZ,float* CameraParameters)
{
float u0,u1,u2,v0,v1,v2,fu,fv,fs,Z1,Z2; // Parameters to describe dots as image points
u1=Xcentre[0];
u2=Xcentre[1];

v1=Ycentre[0];
v2=Ycentre[1];

fu=CameraParameters[0];
fv=CameraParameters[1];
fs=CameraParameters[2];
u0=CameraParameters[3];
v0=CameraParameters[4];

Z1=DepthZ[0];
Z2=DepthZ[1];



Lpts[0][0]=-fu/Z1;
Lpts[0][1]=-fs/Z1;
Lpts[0][2]=-(u0 - u1)/Z1;
Lpts[0][3]=fs + ((u0 - u1)*(v0 - v1))/fv;
Lpts[0][4]=- fu - ((u0 - u1)*(fv*u0 - fv*u1 - fs*v0 + fs*v1))/(fu*fv);
Lpts[0][5]=(fs*(u0 - u1))/fu - ((v0 - v1)*(pow(fs,2) + pow(fu,2)))/(fu*fv);
Lpts[1][0]=0;
Lpts[1][1]=-fv/Z1;
Lpts[1][2]=-(v0 - v1)/Z1;
Lpts[1][3]=fv + pow((v0 - v1),2)/fv;
Lpts[1][4]=(fs*pow((v0 - v1),2))/(fu*fv) - ((u0 - u1)*(v0 - v1))/fu;
Lpts[1][5]=(fv*(u0 - u1))/fu - (fs*(v0 - v1))/fu;
Lpts[2][0]=-fu/Z2;
Lpts[2][1]=-fs/Z2;
Lpts[2][2]=-(u0 - u2)/Z2;
Lpts[2][3]=fs + ((u0 - u2)*(v0 - v2))/fv;
Lpts[2][4]=- fu - ((u0 - u2)*(fv*u0 - fv*u2 - fs*v0 + fs*v2))/(fu*fv);
Lpts[2][5]=(fs*(u0 - u2))/fu - ((v0 - v2)*(pow(fs,2) + pow(fu,2)))/(fu*fv);
Lpts[3][0]=0;
Lpts[3][1]=-fv/Z2;
Lpts[3][2]=-(v0 - v2)/Z2;
Lpts[3][3]=fv + pow((v0 - v2),2)/fv;
Lpts[3][4]=(fs*pow((v0 - v2),2))/(fu*fv) - ((u0 - u2)*(v0 - v2))/fu;
Lpts[3][5]=(fv*(u0 - u2))/fu - (fs*(v0 - v2))/fu;

}


 /*
				FUNCTION: Define DefineFeatureSegment
//******************************************************************************************************************************************************
Inputs: Current dot positions, Xcentre and Ycentre and desired image features s* (vpColVector)
				
Outputs: deltas which is the difference between the current image and the desired image featue

Function by Philip Long
//*******************************************************************************************************************************************************


*/

 void DefineDeltaS(float* Xcentre,float* Ycentre,vpColVector sdesired,vpColVector& deltas)
 {
	float uc,vc,l,theta,AngleDiff,AngleDiff2,theta2;
	// Assign Segment Variables
	uc=(Xcentre[0]+Xcentre[1])/2;
	vc=(Ycentre[0]+Ycentre[1])/2;
	
	l=pow( (pow((Xcentre[0]-Xcentre[1]),2) + pow((Ycentre[0]-Ycentre[1]),2) ), 0.5);
	
	//printf("l used to calculate delts s=%f \n ",l);
	
	theta=atan((Ycentre[0]-Ycentre[1])/(Xcentre[0]-Xcentre[1]));
	theta2=theta+PI; // if blob1 change position with blob two 
	deltas[0]=sdesired[0]-uc;
	deltas[1]=sdesired[1]-vc;
	deltas[2]=sdesired[2]-l;
	deltas[3]=sdesired[3]-theta;


//	AngleDiff=atan2(sin(theta-sdesired[3]), cos(theta-sdesired[3]));
	//AngleDiff2=atan2(sin(theta2-sdesired[3]), cos(theta2-sdesired[3]));
	//deltas[3]=AngleDiff;
	/*
	AngleDiff = sdesired[3] - theta;
	AngleDiff += (AngleDiff>PI) ? -(2*PI) : (AngleDiff<-PI) ? (2*PI) : 0;
	
	AngleDiff2 = sdesired[3] - theta2;
	AngleDiff2 += (AngleDiff2>PI) ? -(2*PI) : (AngleDiff2<-PI) ? (2*PI) : 0;


	if (fabs(AngleDiff)<fabs(AngleDiff2))
	{
		deltas[3]=AngleDiff;

	}
	else
	{
		deltas[3]=AngleDiff2;

	}
	*/
 
 }

/*
				FUNCTION:  Deal out S

*/

 void DefineS(float* Xcentre,float* Ycentre,vpColVector& s)
 {
    s[0]=Xcentre[0];
    s[1]=Ycentre[0];

    s[2]=Xcentre[1];
    s[3]=Ycentre[1];

    s[4]=Xcentre[2];
    s[5]=Ycentre[2];

    s[6]=Xcentre[3];
    s[7]=Ycentre[3];
 }


 /*
				FUNCTION:  KeepImageinView
//******************************************************************************************************************************************************
Inputs: Current dot positions, Xcentre and Ycentre 
				
Outputs: deltas_FOV

If the image features approach a boundary and pass a certain threshold, a deltas is generted that pushes the image away from the boundary


Note as always the notation is "inversed" for image space
           y         (0,640)
 o------------------>
 |
x| 
 |
 |
 |
 v------------------(480,640) 
(480,0)             

Function by Philip Long
//*******************************************************************************************************************************************************


*/ 
 
  void KeepImageinView(float* Xcentre,float* Ycentre,vpColVector& deltas_FOV)
 {
	float Rho=50.0; // The threshold a magic number at this moment
    float xmin=0;
    float xmax=640.0;
    float ymin=0;
    float ymax=480.0; // The image boundaries 
    float etaPos=100.0; // A gain pushing in the positive direction
    float etaNeg=-100.0; // A gain pushing in the positive direction


   if(fabs(Xcentre[0]-xmin)<Rho)
   {
   deltas_FOV[0]= etaPos*pow((1/fabs(Xcentre[0]-xmin))-(1/Rho),2);
   }
   else if(fabs(Xcentre[0]-xmax)<Rho)
   {
   deltas_FOV[0]= etaNeg*pow((1/fabs(Xcentre[0]-xmax))-(1/Rho),2);
   }
   else
   {
    deltas_FOV[0]=0.0;
    }
   // deltas_FOV[1]
   if(fabs(Ycentre[0]-ymin)<Rho)
   {
      deltas_FOV[1]= etaPos*pow((1/fabs(Ycentre[0]-ymin))-(1/Rho),2);
   }
   else if(fabs(Ycentre[0]-ymax)<Rho)
   {
      deltas_FOV[1]= etaNeg*pow((1/fabs(Ycentre[0]-ymax))-(1/Rho),2);
   }
   else
   {
    deltas_FOV[1]=0.0;
    }

   if(fabs(Xcentre[1]-xmin)<Rho)
   {
     deltas_FOV[2]= etaPos*pow((1/fabs(Xcentre[1]-xmin))-(1/Rho),2);
   }
   else if(fabs(Xcentre[1]-xmax)<Rho)
   {
     deltas_FOV[2]= etaNeg*pow((1/fabs(Xcentre[1]-xmax))-(1/Rho),2); 
   }
   else
   {
    deltas_FOV[2]=0.0;
    }

   // deltas_FOV[3]
   if(fabs(Ycentre[1]-ymin)<Rho)
   {
        deltas_FOV[3]= etaPos*pow((1/fabs(Ycentre[1]-ymin))-(1/Rho),2);
   }
   else if(fabs(Ycentre[1]-ymax)<Rho)
   {
        deltas_FOV[3]= etaNeg*pow((1/fabs(Ycentre[1]-ymin))-(1/Rho),2);
   }
   else
   {
    deltas_FOV[3]=0.0;
    }

 }
 
 /*
				FUNCTION: ScrewTransformation
//******************************************************************************************************************************************************
Inputs: 1. Transformation matrix from point j to point i  4 x 4 trasnformation Matrix Tij
		2. Kinematic screw expressed at point origin i in the frame of i, Vii   6x1 Velocity screw		
		
Outputs:  Kinematic screw expressed at point origin j in the frame of j,  Vjj

Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************


*/
 void ScrewTransformation(vpHomogeneousMatrix Tij,vpColVector Vii,vpColVector& Vjj)
{

//Declarations


vpTranslationVector Pij;
vpRotationMatrix Rij;
vpRotationMatrix Rji;// Transpose
vpMatrix Pijskew(3,3);
vpMatrix L(6,6); //Screw Transform
vpMatrix LD(3,3); //submatrix


Tij.extract(Rij);
Tij.extract(Pij);

//Rji=Rij.transpose();

//transpose
for(int i=0;i<3;i++)
		{			
			for(int j=0;j<3;j++)
			{
			Rji[j][i]=Rij[i][j];
			}
		}
			



SkewSym(Pij,Pijskew);
LD=Rji*Pijskew;
// Build Screw Trasnform
for (int i=0;i<3;i++)
		{			
			for(int j=0;j<3;j++)
			{
				L[i][j]=Rji[i][j]; // Upper  left 
				L[i+3][j+3]=Rji[i][j];// Lower right
				L[i][j+3]=-LD[i][j];// Upper right 
				L[i+3][j]=0;// Lower left 
				
			}		

		}
// Change kinematic screw

Vjj=L*Vii;


}


/*	                    FUNCTION: RotateScrew
//******************************************************************************************************************************************************
Inputs: 1.  bV a 6*1 vector in frame b
		2.  wRb a rotation matrix from b to w
		
Outputs:  wV a 6*1 vector in frame w



Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************


*/
void RotateScrew(vpColVector bV, vpColVector& wV, vpRotationMatrix wRb)
{
  vpMatrix trans; // initialise
  trans.eye(6);

  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans[i][j] = wRb[i][j];
      trans[i+3][j+3] = wRb[i][j];
    }
  } 
  wV = trans*bV;
} 


 /*
				FUNCTION: FIFO Buffer
//******************************************************************************************************************************************************
Inputs: 1.  The current buffer ForceBuffer
		      2. The measurement to be addaed to the buffer
		
Outputs:  ForceBuffer

Summary this function places the measurement into the buffer while eliminated the oldsest measurement

Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************


*/
void fifoBufferInsert(vpMatrix& ForceBuffer,vpColVector ResolvedForce)
{
	
	//printf("Number of rows=%d,Columns=%d",ForceBuffer.getRows(),ForceBuffer.getCols());


	for(int i=(ForceBuffer.getRows()-1);i>-1;i--) // From the last row to the first
	{

		for(int j=0;j<ForceBuffer.getCols();j++) // For all columns
		{
			
			if(i>0)
			{
			ForceBuffer[i][j]=ForceBuffer[i-1][j]; // Replace ith row with ith-1 row
			}
			else
			{
			ForceBuffer[i][j]=ResolvedForce[j]; // First row is new measurement
			}
		}
	}

}


 /*
				FUNCTION: LowPassFilter
//******************************************************************************************************************************************************
Inputs: 1.  ForceBuffer

Outputs:  MovingAverageForce

This function simply gets the average of the readings in the buffer to filter out the noise

Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void LowPassFilter(vpMatrix ForceBuffer,float* MovingAverageForce)
{
	float sum;

	for(int i=0;i<6;i++)
	{
		sum=0;
 
		for(int j=0; j<ForceBuffer.getRows(); j++) 
		{
			sum+=ForceBuffer[j][i];
		}

		MovingAverageForce[i]=sum/ForceBuffer.getRows();
	}

}


 /*
				FUNCTION: LowPassFilter
//******************************************************************************************************************************************************
Inputs: 1.  ForceBuffer

Outputs:  MovingAverageForce

This function simply gets the average of the readings in the buffer to filter out the noise

Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void LowPassFilter(vpMatrix ForceBuffer,vpColVector& MovingAverageForce)
{
	float sum;

	for(int i=0;i<6;i++)
	{
		sum=0;
 
		for(int j=0; j<ForceBuffer.getRows(); j++) 
		{
			sum+=ForceBuffer[j][i];
		}

		MovingAverageForce[i]=sum/ForceBuffer.getRows();
	}

}




 /*
				FUNCTION: NormalToSurface
//******************************************************************************************************************************************************
Inputs: 1.  ForceBuffer

Outputs: 2. OmegaF angular velocity used to position tool axis normal to surface

This function using the current value of the resolved force to oreint the tool Z-axis 
so it is normal to the surface

Force variables used to control orientaion
     Anglediff:Angle difference between tool normal=[0 0 -1] (always) and the surface normal
     Axisdiff: Axis is the cross product between tool normal=[0 0 -1] (always) and the surface normal
    Surface normal is calculated from the current readings of the force sensor

Function by (c) 2013 Philip Long


//*******************************************************************************************************************************************************
*/      
void NormalToSurface(float* MovingAverageForce,float* OmegaForce)
{
   
    float Anglediff;
    float Axisdiff[3];
    float normalbyforce[3];
    float sumResolvedForce;

    sumResolvedForce=pow(MovingAverageForce[0],2)+pow(MovingAverageForce[1],2)+pow(MovingAverageForce[2],2);
   // printf("\n(func:NormalToSurface) sumResolvedForce=%f",sumResolvedForce);
	 for(int i=0;i<3;i++)
	 {
		normalbyforce[i]=MovingAverageForce[i]/pow(sumResolvedForce,0.5);
	 }
    if(sumResolvedForce<5) // Only firing if the force is above a certain threshold
    {
        for(int i=0;i<3;i++)
        {
            OmegaForce[i]=0.0;
        }
    }
    else
    {

        Anglediff=acos(-normalbyforce[2]);
        Axisdiff[0]=normalbyforce[1];
        Axisdiff[1]=-normalbyforce[0];
        Axisdiff[2]=0.0;
        for(int i=0;i<3;i++)
        {
            OmegaForce[i]=Axisdiff[i]*Anglediff;
        }
    }
	//printf("FUNCTION:Normal by force in the tool frame=[%f,%f,%f]\n",normalbyforce[0],normalbyforce[1],normalbyforce[2]);

}





 /*
				FUNCTION: LinearTrajq UNTESTED!!!!!!!
//******************************************************************************************************************************************************
Inputs: 1.  qinit and qfinal and a time, 

Outputs:  q(t) a the commanded joint position

This is a simple linear trajectory

Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void LinearTrajq(vpColVector qinit,vpColVector qfinal,vpColVector& qt,float time,float tfinal)
{
	float rt;
	float D[7];

	rt=(10*( pow((time/(tfinal)),3)))-(15*( pow((time/(tfinal)),4)))+(6*(  pow((time/(tfinal)),5)));

	for(int i=0;i<7;i++)
	{
		qt[i]=qinit[i]+rt*(qfinal[i]-qinit[i]);
	}

}


 /*
				FUNCTION: Trajectory in sections
//******************************************************************************************************************************************************
Inputs: 1.  qinit and qfinal and a time, 

Outputs:  q(t) a the commanded joint position


Summary: 

T is a matrix containing the total time between point in first column and rest time in second colum
(Total time=movetime+resttime), the first row does not correspond to any point!

Q is a matrix containing the points on the trajectory.

For example: 
	 
	 T[2][0]=10.0 and  T[2][1]=2.0 and 
	 Q[1][0]=0.0;Q[1][1]=0;Q[1][2]=0;Q[1][3]=0;Q[1][4]=0;Q[1][5]=0;Q[1][6]=0;
	 Q[2][0]=0.0;Q[2][1]=0;Q[2][2]=0;Q[2][3]=0;Q[2][4]=0;Q[2][5]=0;Q[2][6]=PI/2;
	 
The trajectory moves from Q[1][:] --> Q[2][:] in 8 seconds then rests for 2 seconds before commencing
Q[2][:] --> Q[3][:] in time specified by T[3][:]

The initial input is given by qinit, at each instant the trajectory outputs qt the desired joint position

Brief: A List a set of joint configurations Q that the robot attains at times T
	 
Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void TrajecSects(vpColVector qinit,vpColVector& qt,float t)
{


	vpMatrix T(11,2); // Matrix containing all the time variables
	vpMatrix Q(11,7); // Matrix containing the q variables
	
	/*------------------------------------------------------ 
 Matrix Generated by function Mat2Cmatrix 
------------------------------------------------------ */ 
 
	T[0][0]=0.0;		T[0][1]=3.0;
	T[1][0]=100.0;		T[1][1]=10.0;
	T[2][0]=100.0;		T[2][1]=10.0;
	T[3][0]=100.0;		T[3][1]=10.0;
	T[4][0]=100.0;		T[4][1]=10.0;
	T[5][0]=100.0;		T[5][1]=10.0;
	T[6][0]=100.0;		T[6][1]=10.0;
	T[7][0]=100.0;		T[7][1]=10.0;
	T[8][0]=100.0;		T[8][1]=10.0;
	T[9][0]=100.7;		T[9][1]=10.0;
	T[10][0]=100.8;  	T[10][1]=10.0;
	
	 /*
    Q[0][0]=0.0;    Q[0][1]=0.0;    Q[0][2]=0.0;    Q[0][3]=-PI/2;    Q[0][4]=0.0;      Q[0][5]=PI/4;    Q[0][6]=3*PI/4;
    Q[1][0]=0.0;    Q[1][1]=0.0;    Q[1][2]=0.0;    Q[1][3]=-PI/2;    Q[1][4]=PI/4;     Q[1][5]=PI/2;    Q[1][6]=-3*PI/4;
    Q[2][0]=0.0;    Q[2][1]=0.0;    Q[2][2]=0.0;    Q[2][3]=-PI/2;    Q[2][4]=-PI/4;    Q[2][5]=PI/4;    Q[2][6]=3*PI/4;
    Q[3][0]=0.0;    Q[3][1]=0.0;    Q[3][2]=0.0;    Q[3][3]=-PI/2;    Q[3][4]=PI/4;     Q[3][5]=0.0;     Q[3][6]=-3*PI/4;
    Q[4][0]=0.0;    Q[4][1]=0.0;    Q[4][2]=0.0;    Q[4][3]=-PI/2;    Q[4][4]=-PI/4;    Q[4][5]=PI/4;    Q[4][6]=3*PI/4;
    Q[5][0]=0.0;    Q[5][1]=0.0;    Q[5][2]=0.0;    Q[5][3]=-PI/2;    Q[5][4]=0.0;      Q[5][5]=PI/2;    Q[5][6]=3*PI/4;
    Q[6][0]=0.0;    Q[6][1]=0.0;    Q[6][2]=0.0;    Q[6][3]=-PI/2;    Q[6][4]=0.0;      Q[6][5]=0.0;     Q[6][6]=0.0;
    Q[7][0]=0.0;    Q[7][1]=0.0;    Q[7][2]=0.0;    Q[7][3]=-PI/2;    Q[7][4]=0.0;      Q[7][5]=0.0;     Q[7][6]=0.0;
    Q[8][0]=0.0;    Q[8][1]=0.0;    Q[8][2]=0.0;    Q[8][3]=-PI/2;    Q[8][4]=0.0;      Q[8][5]=0.0;     Q[8][6]=0.0;
    Q[9][0]=0.0;    Q[9][1]=0.0;    Q[9][2]=0.0;    Q[9][3]=-PI/2;    Q[9][4]=0.0;      Q[9][5]=0.0;     Q[9][6]=0.0;
    Q[10][0]=0.0;   Q[10][1]=0.0;   Q[10][2]=0.0;   Q[10][3]=0.0;     Q[10][4]=0.0;     Q[10][5]=0.0;    Q[10][6]=0.0;
*/
    Q[0][0]=qinit[0];    Q[0][1]=qinit[1];    Q[0][2]=qinit[2];    Q[0][3]=qinit[3];    Q[0][4]=qinit[4]+PI/8;     Q[0][5]=qinit[5];    Q[0][6]=qinit[6]+PI/12;
    Q[1][0]=qinit[0];    Q[1][1]=qinit[1];    Q[1][2]=qinit[2];    Q[1][3]=qinit[3];    Q[1][4]=qinit[4]-PI/8;     Q[1][5]=qinit[5];    Q[1][6]=qinit[6]-PI/12;
    Q[2][0]=qinit[0];    Q[2][1]=qinit[1];    Q[2][2]=qinit[2];    Q[2][3]=qinit[3];    Q[2][4]=qinit[4];     Q[2][5]=qinit[5]-PI/6;    Q[2][6]=qinit[6];
    Q[3][0]=qinit[0];    Q[3][1]=qinit[1];    Q[3][2]=qinit[2];    Q[3][3]=qinit[3];    Q[3][4]=qinit[4];     Q[3][5]=qinit[5]+PI/6;     Q[3][6]=qinit[6]+PI/4;
    Q[4][0]=qinit[0];    Q[4][1]=qinit[1];    Q[4][2]=qinit[2];    Q[4][3]=qinit[3];    Q[4][4]=qinit[4]+PI/8;      Q[4][5]=qinit[5]-PI/6;    Q[4][6]=qinit[6]+PI/6;
    Q[5][0]=qinit[0];    Q[5][1]=qinit[1];    Q[5][2]=qinit[2];    Q[5][3]=qinit[3];    Q[5][4]=qinit[4]-PI/8;      Q[5][5]=qinit[5]+PI/6;    Q[5][6]=qinit[6];
    Q[6][0]=qinit[0];    Q[6][1]=qinit[1];    Q[6][2]=qinit[2];    Q[6][3]=qinit[3];    Q[6][4]=qinit[4];      Q[6][5]=qinit[5];     Q[6][6]=qinit[6];
    Q[7][0]=qinit[0];    Q[7][1]=qinit[1];    Q[7][2]=qinit[2];    Q[7][3]=qinit[3];    Q[7][4]=qinit[4];      Q[7][5]=qinit[5];     Q[7][6]=qinit[6];


    Q[8][0]=0.0;    Q[8][1]=0.0;    Q[8][2]=0.0;    Q[8][3]=-PI/2;    Q[8][4]=0.0;      Q[8][5]=0.0;     Q[8][6]=0.0;
    Q[9][0]=0.0;    Q[9][1]=0.0;    Q[9][2]=0.0;    Q[9][3]=-PI/2;    Q[9][4]=0.0;      Q[9][5]=0.0;     Q[9][6]=0.0;
    Q[10][0]=0.0;   Q[10][1]=0.0;   Q[10][2]=0.0;   Q[10][3]=0.0;     Q[10][4]=0.0;     Q[10][5]=0.0;    Q[10][6]=0.0;

	float rt,RestTime,time,tn,tsection,tfinal;
	float Sum1,Sum2;
	vpColVector qi(7);
	vpColVector qf(7);
	int Section,PassSection;
	float D[7];
	


	for(int kk=0; kk<T.getRows();kk++)
	{
		
		Sum1=0; // For part one
		
		for(int i=0;i<kk;i++) // might have to add Section +1 
		{
		Sum1=Sum1+T[i][0];
		}

		Sum2=0; // For part two

		for(int i=0;i<(kk+1);i++) // might have to add Section +1 
		{
		Sum2=Sum2+T[i][0];
		}


		if (t>=Sum1 && t<Sum2)
		{
		Section=kk-1; // since we ignore the first case
		}
		else if( t==0)
		{
		Section=0; 
		}
		
	}

	tsection=T[Section+1][0]; // Total time per section including rest
	RestTime=T[Section+1][1]; // Total Rest time between points  

	
	Sum1=0;
	for(int i=0;i<(Section+1);i++) // might have to add Section +1 
	{
	Sum1=Sum1+T[i][0];
	}
	tn=t-Sum1;

	time=fmod(tn,tsection); // t= 0 --> tsection then resets
	PassSection=Section-1;


	if (PassSection==-1) // Initial Section
	{
	
	for(int i=0;i<7;i++)
		{	
		qi[i]=qinit[i];
		qf[i]=Q[PassSection+1][i];
		}

	}
	else if( PassSection > Q.getRows()-1) // Final Point
	{
	for(int i=0;i<7;i++)
		{
		qi[i]=Q[Q.getRows()-1][i];
		qf[i]=Q[Q.getRows()-1][i];
		}
	}
	else
	{
	for(int i=0;i<7;i++)
		{
		qi[i]=Q[PassSection][i];
		qf[i]=Q[PassSection+1][i];
		}
	}


        if( time>(tsection-RestTime)) // A rest time between each trajectory section is defined
	{
            time=tsection-RestTime;
	    time=tsection-RestTime;

	}

   rt=(    10*(pow((time/(tsection-RestTime)),3)))    -(15*( pow((time/(tsection-RestTime)),4)))   + (6*(pow((time/(tsection-RestTime)),5))    );


	for(int i=0;i<7;i++)
	{
	qt[i]=qi[i]+rt*(qf[i]-qi[i]);
	}
	


}




 /*
				FUNCTION: Trajectory in sections to move to a number of predefined 3D points
//******************************************************************************************************************************************************
Inputs: 1.  Xinit and a time, 

Outputs:  X(t) a the commanded cartesian position


Summary: 

T is a matrix containing the total time between point in first column and rest time in second colum
(Total time=movetime+resttime), the first row does not correspond to any point!

Q is a matrix containing the points on the trajectory.

For example: 
	 
	 T[2][0]=10.0 and  T[2][1]=2.0 and 
	 Q[1][0]=0.0;Q[1][1]=0;Q[1][2]=0;Q[1][3]=0;Q[1][4]=0;Q[1][5]=0;Q[1][6]=0;
	 Q[2][0]=0.0;Q[2][1]=0;Q[2][2]=0;Q[2][3]=0;Q[2][4]=0;Q[2][5]=0;Q[2][6]=PI/2;
	 
The trajectory moves from Q[1][:] --> Q[2][:] in 8 seconds then rests for 2 seconds before commencing
Q[2][:] --> Q[3][:] in time specified by T[3][:]

The initial input is given by qinit, at each instant the trajectory outputs qt the desired joint position

Brief: A List a set of joint configurations Q that the robot attains at times T
	 
Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void XTrajec(vpHomogeneousMatrix bMt_init,vpHomogeneousMatrix& bMt,float t,vpColVector& bVt,vpColVector& bOmegat)
{


vpMatrix T(11,3); // Matrix containing all the time variables
vpMatrix Q(11,7); // Matrix containing the q variables
vpColVector V(3);
vpColVector Omega(3);
vpTranslationVector bPt,Pi,Pf,Pt,Pinit,Pfinal;
vpRotationMatrix bRt,Rinit,Rf,Ri,Rt,RotuAlpha,RotuAlpha_rt,RiT;
vpQuaternionVector rQi,rQf;
float rQi_norm,rQf_norm;
vpThetaUVector uAlpha;
float nu;
vpColVector u(3);
vpMatrix uskew(3,3);
vpMatrix I(3,3);
vpMatrix A(3,3);
vpMatrix B(3,3);
vpMatrix C(3,3);
double alpha;
float rt,rtdot,RestTime,time,tn,tsection,tfinal;
float Sum1,Sum2;
int Section,PassSection;



bMt_init.extract(Pinit);
bMt_init.extract(Rinit);	

T[0][0]=0; T[0][1]=0.3;T[0][2]=0;
T[1][0]=50.0;T[1][1]=10;T[1][2]=0;
T[2][0]=100.0;T[2][1]=10;T[2][2]=1;
T[3][0]=100.0;T[3][1]=10;T[3][2]=0;
T[4][0]=100.0;T[4][1]=10;T[4][2]=0;
T[5][0]=100.0;T[5][1]=10;T[5][2]=1;
T[6][0]=100;T[6][1]=10;T[6][2]=0;
T[7][0]=100;T[7][1]=10;T[7][2]=0;
T[8][0]=100;T[8][1]=10;T[8][2]=1;
T[9][0]=100;T[9][1]=10;T[9][2]=0;
T[10][0]=100;T[10][1]=0.3;T[10][2]=0;
	
/* This Q is just a test
Q[0][0]=-0.35797;  Q[0][1]=0;  Q[0][2]=0.25;  Q[0][3]=0.924277;  Q[0][4]=-0.381628;  Q[0][5]=0.003943;  Q[0][6]=0.007521;
Q[1][0]=-0.55;     Q[1][1]=0;  Q[1][2]=0.15;  Q[1][3]=0.924277;  Q[1][4]=-0.381628;  Q[1][5]=0.003943;  Q[1][6]=0.007521;
Q[2][0]=-0.35797;  Q[2][1]=0;  Q[2][2]=0.25;  Q[2][3]=0.924277;  Q[2][4]=-0.381628;  Q[2][5]=0.003943;  Q[2][6]=0.007521;
Q[3][0]=-0.35797;  Q[3][1]=0;  Q[3][2]=0.15;  Q[3][3]=0.924277;  Q[3][4]=-0.381628;  Q[3][5]=0.003943;  Q[3][6]=0.007521;
Q[4][0]=-0.35797;  Q[4][1]=0;  Q[4][2]=0.15;  Q[4][3]=0;  Q[4][4]=1;  Q[4][5]=0;  Q[4][6]=0;
Q[5][0]=-0.35797;  Q[5][1]=0;  Q[5][2]=0.15;  Q[5][3]=0;  Q[5][4]=1;  Q[5][5]=0;  Q[5][6]=0;
Q[6][0]=-0.35797;  Q[6][1]=0;  Q[6][2]=0.15;  Q[6][3]=0;  Q[6][4]=1;  Q[6][5]=0;  Q[6][6]=0;
Q[7][0]=-0.35797;  Q[7][1]=0;  Q[7][2]=0.15;  Q[7][3]=0;  Q[7][4]=1;  Q[7][5]=0;  Q[7][6]=0;
Q[8][0]=-0.35797;  Q[8][1]=0;  Q[8][2]=0.15;  Q[8][3]=0;  Q[8][4]=1;  Q[8][5]=0;  Q[8][6]=0;
Q[9][0]=-0.35797;  Q[9][1]=0;  Q[9][2]=0.15;  Q[9][3]=0;  Q[9][4]=1;  Q[9][5]=0;  Q[9][6]=0;
*/

//
Q[0][0]=-0.500;  Q[0][1]=0.08516;  Q[0][2]=0.107;  Q[0][3]=0.924277;  Q[0][4]=-0.381628;  Q[0][5]=0.003943;  Q[0][6]=0.007521;
Q[1][0]=-0.500;     Q[1][1]=0.08516;  Q[1][2]=0.107;  Q[1][3]=0.924277;  Q[1][4]=-0.381628;  Q[1][5]=0.003943;  Q[1][6]=0.007521;
Q[2][0]=-0.35797;  Q[2][1]=0;  Q[2][2]=0.25;  Q[2][3]=0.924277;  Q[2][4]=-0.381628;  Q[2][5]=0.003943;  Q[2][6]=0.007521;
Q[3][0]=-0.35797;  Q[3][1]=0;  Q[3][2]=0.15;  Q[3][3]=0.924277;  Q[3][4]=-0.381628;  Q[3][5]=0.003943;  Q[3][6]=0.007521;
Q[4][0]=-0.35797;  Q[4][1]=0;  Q[4][2]=0.15;  Q[4][3]=0;  Q[4][4]=1;  Q[4][5]=0;  Q[4][6]=0;
Q[5][0]=-0.35797;  Q[5][1]=0;  Q[5][2]=0.15;  Q[5][3]=0;  Q[5][4]=1;  Q[5][5]=0;  Q[5][6]=0;
Q[6][0]=-0.35797;  Q[6][1]=0;  Q[6][2]=0.15;  Q[6][3]=0;  Q[6][4]=1;  Q[6][5]=0;  Q[6][6]=0;
Q[7][0]=-0.35797;  Q[7][1]=0;  Q[7][2]=0.15;  Q[7][3]=0;  Q[7][4]=1;  Q[7][5]=0;  Q[7][6]=0;
Q[8][0]=-0.35797;  Q[8][1]=0;  Q[8][2]=0.15;  Q[8][3]=0;  Q[8][4]=1;  Q[8][5]=0;  Q[8][6]=0;
Q[9][0]=-0.35797;  Q[9][1]=0;  Q[9][2]=0.15;  Q[9][3]=0;  Q[9][4]=1;  Q[9][5]=0;  Q[9][6]=0;	


	for(int kk=0; kk<T.getRows();kk++)
	{
		
		Sum1=0; // For part one
		
		for(int i=0;i<kk;i++) // might have to add Section +1 
		{
		Sum1=Sum1+T[i][0];
		}

		Sum2=0; // For part two

		for(int i=0;i<(kk+1);i++) // might have to add Section +1 
		{
		Sum2=Sum2+T[i][0];
		}


		if (t>=Sum1 && t<Sum2)
		{
		Section=kk-1; // since we ignore the first case
		}
		else if( t==0)
		{
		Section=0; 
		}
		
	}

	tsection=T[Section+1][0]; // Total time per section including rest
	RestTime=T[Section+1][1]; // Total Rest time between points  

	
	Sum1=0;
	for(int i=0;i<(Section+1);i++) // might have to add Section +1 
	{
	Sum1=Sum1+T[i][0];
	}
	tn=t-Sum1;

	time=fmod(tn,tsection); // t= 0 --> tsection then resets
	PassSection=Section-1;

// In this Part we define the start and end points of each section

	if (PassSection==-1) // Initial Section
    //----------------------------
	{	
	    for(int i=0;i<3;i++) // Extract the Pi and Pf positions
	    {	
		    Pi[i]=Pinit[i];
		    Pf[i]=Q[PassSection+1][i];
	    }
          
	    for(int i=3;i<7;i++) // Extract the Quaternion Vector
	    {	
	        rQf[i-3]=Q[PassSection+1][i];
            
	    }
        
        
        Ri=Rinit;
        
        // have to normailse the vector
       rQf_norm=pow((rQf[0]*rQf[0])+(rQf[1]*rQf[1])+(rQf[2]*rQf[2]+(rQf[3]*rQf[3])),0.5);
       for(int i=0;i<4;i++) // Extract the Quaternion Vector
	    {	
	        rQf[i]=rQf[i]/(rQf_norm);     
	    }
        Rf=vpRotationMatrix(rQf);        

    }
    //----------------------------
    else if( PassSection > Q.getRows()-1) // Final Point
    //----------------------------
    {

	    for(int i=0;i<3;i++) // Extract the Pi and Pf positions
	    {	
		    Pi[i]=Q[Q.getRows()-1][i];
		    Pf[i]=Q[Q.getRows()-1][i];
	    }
          
	    for(int i=3;i<7;i++) // Extract the Quaternion Vector
	    {
            rQi[i-3]=Q[Q.getRows()-1][i];	
	        rQf[i-3]=Q[Q.getRows()-1][i];
	    }


        rQf_norm=pow((rQf[0]*rQf[0])+(rQf[1]*rQf[1])+(rQf[2]*rQf[2]+(rQf[3]*rQf[3])),0.5);
        rQi_norm=pow((rQi[0]*rQi[0])+(rQi[1]*rQi[1])+(rQi[2]*rQi[2]+(rQi[3]*rQi[3])),0.5);
	    for(int i=0;i<4;i++) // Extract the Quaternion Vector
	    {	
	        rQf[i]=rQf[i]/(rQf_norm);
            rQi[i]=rQi[i]/(rQi_norm);
	    }

       //  printf("PassSection=%d",PassSection);
        Ri=vpRotationMatrix(rQi);
        Rf=vpRotationMatrix(rQf);

    }

    else // Standard Point
    {
  
        for(int i=0;i<3;i++) // Extract the Pi and Pf positions
	    {	
		    Pi[i]=Q[PassSection][i];
		    Pf[i]=Q[PassSection+1][i];
	    }
          
	    for(int i=3;i<7;i++) // Extract the Quaternion Vector
	    {
            rQi[i-3]=Q[PassSection][i];	
	        rQf[i-3]=Q[PassSection+1][i];
	    }

        
        rQf_norm=pow((rQf[0]*rQf[0])+(rQf[1]*rQf[1])+(rQf[2]*rQf[2]+(rQf[3]*rQf[3])),0.5);
        rQi_norm=pow((rQi[0]*rQi[0])+(rQi[1]*rQi[1])+(rQi[2]*rQi[2]+(rQi[3]*rQi[3])),0.5);
	    for(int i=0;i<4;i++) // Extract the Quaternion Vector
	    {	
	        rQf[i]=rQf[i]/(rQf_norm);
            rQi[i]=rQi[i]/(rQi_norm);
	    }
     //   printf("PassSection=%d",PassSection);
        Ri=vpRotationMatrix(rQi);
        Rf=vpRotationMatrix(rQf);
    }


// Define rt the interpolator


    if( time>(tsection-RestTime)) // A rest time between each trajectory section is defined
	{
        time=tsection-RestTime;
	}
        rt=(    10*(pow((time/(tsection-RestTime)),3)))    -(15*( pow((time/(tsection-RestTime)),4)))   + (6*(pow((time/(tsection-RestTime)),5))    );
        rtdot =( 30*pow(time,2) )/(pow((tsection-RestTime),3)) -  (60*pow(time,3))/(pow((tsection-RestTime),4)) +   (30*pow(time,4))/(pow((tsection-RestTime),5));



// Define Rt
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            RiT[i][j]=Ri[j][i];
        }
    }
    
    RotuAlpha=Rf*RiT; // Takes the difference between the intial and final points of a section in oreintation 

    //printf("RotuAlpha=\n");printfM(RotuAlpha); 

   // printf("\n rQf=");printfM(rQf);
  //  printf("\nRotuAlpha=");printfM(RotuAlpha);
    uAlpha.buildFrom(RotuAlpha); // u theta Representation of Orientation
   // printf("\nuAlpha=%f,%f,%f",uAlpha[0],uAlpha[1],uAlpha[2]);
    alpha = sqrt(pow(uAlpha[0],2)+pow(uAlpha[1],2)+pow(uAlpha[2],2));
  ///  printf("\n alpha=%f\n",alpha);

     if(fabs(alpha)<0.001) // Set to identity
     {   
     
        
        for(int i=0;i<3;i++)
        {            
            for(int j=0;j<3;j++)
            {
                if(i==j)
                {
                RotuAlpha_rt[i][j]=1.0;
                }
                else
                {
                RotuAlpha_rt[i][j]=0.0;                    
                }
            }
        }

        u[0]=0.0;
        u[1]=0.0;
        u[2]=0.0;
    }
    else
    {
        u[0]=uAlpha[0]/alpha;
        u[1]=uAlpha[1]/alpha;
        u[2]=uAlpha[2]/alpha;
        SkewSym(u,uskew);
        nu=rt*alpha; 
        I.eye(3);A.eye(3,3);B.eye(3,3);C.eye(3,3);

        for(int i=0;i<3;i++)
        {            
            for(int j=0;j<3;j++)
            {
                A[i][j]=u[i]*u[j]*(1-cos(nu));
                B[i][j]=I[i][j]*cos(nu);
                C[i][j]=uskew[i][j]*sin(nu);
                RotuAlpha_rt[i][j]=A[i][j]+B[i][j]+C[i][j];
            }
        }
    }

   
   // printf("\nRotuAlpha_rt=");printfM(RotuAlpha);
    Rt=RotuAlpha_rt*Ri;
   // printf("\nRt=");printfM(Rt);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j += 1)
        {
          bMt[i][j]=Rt[i][j];
        }
    }

	for(int i=0;i<3;i++)
	{       
    	Pt[i]=Pi[i]+rt*(Pf[i]-Pi[i]);
        bMt[i][3]=Pt[i];
        V[i]=rtdot*(Pf[i]-Pi[i]);
        Omega[i]=(u[i]*rtdot*alpha);
        bVt[i]=V[i];
        bOmegat[i]=Omega[i];
	}


}






 /*
				FUNCTION: XTrajecSim to move to a desired position
//******************************************************************************************************************************************************
Inputs: 1.  Xfinal and a time, 

Outputs:  X(t) a the commanded 3D position


Summary: 

Genertes a trajectory for an desired point, like the above except 
	 
Function by (c) 2013 Philip Long
//*******************************************************************************************************************************************************

*/


void XTrajecSim(vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtfinal,vpHomogeneousMatrix& bMt,float time,float tf,vpColVector& bVt,vpColVector& bOmegat)
{


vpColVector V(3);
vpColVector Omega(3);
vpTranslationVector bPt,Pf,Pt,Pinit,Pfinal;
vpRotationMatrix bRt,Rinit,Rfinal,Rt,RotuAlpha,RotuAlpha_rt,RiT;
vpQuaternionVector rQi,rQf;
float rQi_norm,rQf_norm;
vpThetaUVector uAlpha;
float nu;
vpColVector u(3);
vpMatrix uskew(3,3);
vpMatrix I(3,3);
vpMatrix A(3,3);
vpMatrix B(3,3);
vpMatrix C(3,3);
double alpha;
float rt,rtdot;




bMtinit.extract(Pinit);
bMtinit.extract(Rinit);	
bMtfinal.extract(Pfinal);
bMtfinal.extract(Rfinal);

if (time>tf)
{
    bMt=bMtfinal;
	for(int i=0;i<3;i++)
	{       
        V[i]=0.0;
        Omega[i]=0.0;
        bVt[i]=V[i];
        bOmegat[i]=Omega[i];
	}
    return;
}

//rt=(    10*(pow((time/(tf)),3)))    -(15*( pow((time/(tf)),4)))   + (6*(pow((time/(tf)),5))    );
//rtdot =( 30*pow(time,2) )/(pow((tf),3)) -  (60*pow(time,3))/(pow((tf),4)) +   (30*pow(time,4))/(pow((tf),5));

rt= time/(tf);
rtdot =1.0;



// Define Rt
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            RiT[i][j]=Rinit[j][i];
        }
    }
    
    RotuAlpha=Rfinal*RiT; // Takes the difference between the intial and final points of a section in oreintation 
    uAlpha.buildFrom(RotuAlpha); // u theta Representation of Orientation
    alpha = sqrt(pow(uAlpha[0],2)+pow(uAlpha[1],2)+pow(uAlpha[2],2));

     if(fabs(alpha)<0.001) // Set to identity
     {     
        for(int i=0;i<3;i++)
        {            
            for(int j=0;j<3;j++)
            {
                if(i==j)
                {
                RotuAlpha_rt[i][j]=1.0;
                }
                else
                {
                RotuAlpha_rt[i][j]=0.0;                    
                }
            }
        }

        u[0]=0.0;
        u[1]=0.0;
        u[2]=0.0;
    }
    else
    {
        u[0]=uAlpha[0]/alpha;
        u[1]=uAlpha[1]/alpha;
        u[2]=uAlpha[2]/alpha;
        SkewSym(u,uskew);
        nu=rt*alpha; 
        I.eye(3);A.eye(3,3);B.eye(3,3);C.eye(3,3);

        for(int i=0;i<3;i++)
        {            
            for(int j=0;j<3;j++)
            {
                A[i][j]=u[i]*u[j]*(1-cos(nu));
                B[i][j]=I[i][j]*cos(nu);
                C[i][j]=uskew[i][j]*sin(nu);
                RotuAlpha_rt[i][j]=A[i][j]+B[i][j]+C[i][j];
            }
        }
    }

    Rt=RotuAlpha_rt*Rinit;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j += 1)
        {
          bMt[i][j]=Rt[i][j];
        }
    }

	for(int i=0;i<3;i++)
	{       
    	Pt[i]=Pinit[i]+rt*(Pfinal[i]-Pinit[i]);
        bMt[i][3]=Pt[i];
        V[i]=rtdot*(Pfinal[i]-Pinit[i]);
        Omega[i]=(u[i]*rtdot*alpha);
        bVt[i]=V[i];
        bOmegat[i]=Omega[i];
	}


}






/*------------------------------------------------------------------------------------
 

            polyval

This function evaulates a polynomial of any order at x

Input: float P of coeffcicients, n the order to polynomial, x the e
    

*/


float polyval(float* p,int n,float Xt)
{
float y=0.0;

for(int i=0;i<n+1;i++)
{
y=y+(p[i]*pow(Xt,i));
}

return y;
}

/*------------------------------------------------------------------------------------
 
            polyder

This function diffeerntiates a polynomial of any order and returns the coefficients of the new polynomial

Input: float P of coeffcicients, n the order to polynomial
    
*/

void polyder(float* p,int n,float* y)
{

for(int i=1;i<n+1;i++)
{

y[i-1]=i*p[i];
}

}


/*------------------------------------------------------------------------------------
 
            CurveTraject

This function takes a polyomial curve and creates a 5 degree in time trajectory 
along this curve

    // Inputs: Polynomial,time,Initial position, Final Position
    // Outputs: Current Position, Current Velocity
    
------------------------------------------------------------------------------------*/


void CurveTraject(float* p,int n,float time,float tf,vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtFinal,vpHomogeneousMatrix& bMttraj,vpColVector& bVt)
{
float rt,rtdot;
float Yt,Ydot,theta,Xt,Xdot;
float Xfinal,Xinitial;
float dpdx[n-1];
polyder(p,n,dpdx);
vpRotationMatrix R,Rtraj;
vpRotationMatrix Rtheta;

// Since the polynomial is defined in the x-y plane the rotation of theta
// is necessarly in the z axis
R[0][0]=-1.0; R[0][1]=0.0; R[0][2]=0.0;
R[1][0]=0.0; R[1][1]=1.0; R[1][2]=0.0;
R[2][0]=0.0; R[2][1]=0.0; R[2][2]=-1.0;
// Inital Values
// Final Values
Xinitial=bMtinit[0][3];
Xfinal=bMtFinal[0][3];

   
rt=(10*(pow((time/(tf)),3)))    -(15*( pow((time/(tf)),4)))   + (6*(pow((time/(tf)),5))    );
rtdot =(30*pow(time,2) )/(pow((tf),3)) -  (60*pow(time,3))/(pow((tf),4)) +   (30*pow(time,4))/(pow((tf),5));

// Calculate X Y and theta from the polynomial

// X value- X and X dot
Xt=rt*(Xfinal-Xinitial)+Xinitial;
Xdot=rtdot*(Xfinal-Xinitial);

// Y value- Y and Y dot
Yt=polyval(p,2,Xt);
Ydot=polyval(dpdx,1,Xt)*Xdot;

// Theta values- theta and theta dot
theta=polyval(dpdx,1,Xt);
Rtheta[0][0]=cos(-theta); Rtheta[0][1]=-sin(-theta); Rtheta[0][2]=0.0;
Rtheta[1][0]=sin(-theta); Rtheta[1][1]=cos(-theta); Rtheta[1][2]=0.0;
Rtheta[2][0]=0.0; Rtheta[2][1]=0.0; Rtheta[2][2]=1.0;
Rtraj=R*Rtheta;    

// Contruct the output First the position
bMttraj[0][3]=Xt;
bMttraj[1][3]=Yt;
bMttraj[2][3]=bMtinit[2][3]; // Z does not change

for(int i=0;i<3;i++)
{
    for(int j=0;j<3;j++)
    {    

    bMttraj[i][j]=Rtraj[i][j];
    }
}

// Construct the output the speed
bVt[0]=Xdot;bVt[1]=Ydot;
for(int i=2;i<6;i++)
{
    bVt[i]=0.0; 
}
// Output is Pdesired and Rdesired 

}


/*------------------------------------------------------------------------------------
 
            CurveGen

This function takes a polyomial curve and creates a 5 degree in time trajectory 
along this curve

    // Inputs:  Polynomial: p of order n
                Current time: time
                Final time: tf
                Initial position: bMinit
                Final Position:   bMfinal 
    // Outputs: CurveParameters
                CurveParameters[0]=Xt; current Xvalue of curve
                CurveParameters[1]=Xdot;
                CurveParameters[2]=Yt; current Y value of curve
                CurveParameters[3]=Ydot;
                CurveParameters[4]=theta; tangent of curve
    

------------------------------------------------------------------------------------*/

void CurveGen(float* p,int n,float time,float tf,vpHomogeneousMatrix bMtinit,vpHomogeneousMatrix bMtFinal,vpColVector& CurveParameters)
{

float rt,rtdot;
float Yt,Ydot,theta,Xt,Xdot;
float Xfinal,Xinitial;
float dpdx[n-1];
polyder(p,n,dpdx);
vpRotationMatrix R,Rtraj;
vpRotationMatrix Rtheta;

// Inital Values
// Final Values
Xinitial=bMtinit[0][3];
Xfinal=bMtFinal[0][3];
   
rt=(10*(pow((time/(tf)),3)))    -(15*( pow((time/(tf)),4)))   + (6*(pow((time/(tf)),5))    );
rtdot =(30*pow(time,2) )/(pow((tf),3)) -  (60*pow(time,3))/(pow((tf),4)) +   (30*pow(time,4))/(pow((tf),5));

// Trying linear
rt=time/(tf);
// Calculate X Y and theta from the polynomial

// X value- X and X dot
Xt=rt*(Xfinal-Xinitial)+Xinitial;
//Xdot=rtdot*(Xfinal-Xinitial);
Xdot=0.0;

// Y value- Y and Y dot
Yt=polyval(p,2,Xt);
Ydot=polyval(dpdx,1,Xt)*Xdot;

// Theta values- theta and theta dot
theta=polyval(dpdx,1,Xt);

// Output Curve Parameters
CurveParameters[0]=Xt;
CurveParameters[1]=Xdot;
CurveParameters[2]=Yt;
CurveParameters[3]=Ydot;
CurveParameters[4]=theta;
}

/*------------------------------------------------------------------------------------
 
            CurveVelocity part (a)

This function takes a polyomial curve and creates a 5 degree in time trajectory 
along this curve

    // Inputs:  Current position bMt
                CurveParameters
                CurveParameters[0]=Xt; current Xvalue of curve
                CurveParameters[1]=Xdot;
                CurveParameters[2]=Yt; current Y value of curve
                CurveParameters[3]=Ydot;
                CurveParameters[4]=theta; tangent of curve
    // Outputs: bVt Tool velocity in base fraem
    
------------------------------------------------------------------------------------*/


void CurveVelocity(vpHomogeneousMatrix bMt,float CutDepth, vpColVector CurveParameters,vpColVector& bVt)
{

// Declarations
float Yt,Ydot,theta,Xt,Xdot,Zt;
vpMatrix H(7,6);
float N1[3];float N2[3];float N3[3];
float TangentDirection[2];
vpRotationMatrix bRt;
float denom;
vpTranslationVector bPt,bPt2,tPt2;
vpColVector dr(7);

// Extract input data
Xt=CurveParameters[0];
Xdot=CurveParameters[1];
Yt=CurveParameters[2];
Ydot=CurveParameters[3];
theta=CurveParameters[4];
bMt.extract(bRt);bMt.extract(bPt);
Zt=CutDepth; // Zt t is un controlled

// Define a second point on the tool frame
tPt2[0]=0.0;tPt2[1]=0.01;tPt2[2]=0.00;
// Transform into the base frame

//  Transform the position vector to base frame
bPt2=bRt*tPt2;
for(int i=0;i<3;i++)
{
bPt2[i]=bMt[i][0]*tPt2[0]+bMt[i][1]*tPt2[1]+bMt[i][2]*tPt2[2]+bMt[i][3];
}



// Define Tangent, magic number here makes no difference to final direction
TangentDirection[0]=(Xt+0.1)-Xt;
TangentDirection[1]=(theta*((Xt+0.1)-Xt)+Yt)-Yt;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;

// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

dr[0]=N1[0]*(Xt - bPt[0]) + N1[1]*(Yt - bPt[1]) + N1[2]*(Zt - bPt[2]);
dr[1]=- N1[0]*(bPt2[0] - Xt) - N1[1]*(bPt2[1] - Yt) - N1[2]*(bPt2[2] - Zt);
dr[2]=N2[0]*(Xt - bPt[0]) + N2[1]*(Yt - bPt[1]) + N2[2]*(Zt - bPt[2]);
dr[3]=- N2[0]*(bPt2[0] - Xt) - N2[1]*(bPt2[1] - Yt) - N2[2]*(bPt2[2] - Zt);
dr[4]=Xt - bPt[0];
dr[5]=Yt - bPt[1];
dr[6]=Zt - bPt[2];


printf("\n dr=");
printfM(dr);
printf("bPt2=%f,%f,%f \n",bPt2[0],bPt2[1],bPt2[2]);
printf("N1=%f,%f,%f \n",N1[0],N1[1],N1[2]);

// Define H
H[0][0]=N1[0];H[0][1]=N1[1];H[0][2]=N1[2];H[0][3]=0;H[0][4]=0;H[0][5]=0;
H[1][0]=N1[0];H[1][1]=N1[1];H[1][2]=N1[2];


H[1][3]=tPt2[1]*(N1[0]*bRt[0][2] + N1[1]*bRt[1][2] + N1[2]*bRt[2][2]) - tPt2[2]*(N1[0]*bRt[0][1] + N1[1]*bRt[1][1] + N1[2]*bRt[2][1]);
H[1][4]=tPt2[2]*(N1[0]*bRt[0][0] + N1[1]*bRt[1][0] + N1[2]*bRt[2][0]) - tPt2[0]*(N1[0]*bRt[0][2] + N1[1]*bRt[1][2] + N1[2]*bRt[2][2]);
H[1][5]=tPt2[0]*(N1[0]*bRt[0][1] + N1[1]*bRt[1][1] + N1[2]*bRt[2][1]) - tPt2[1]*(N1[0]*bRt[0][0] + N1[1]*bRt[1][0] + N1[2]*bRt[2][0]);

H[2][0]=N2[0];H[2][1]=N2[1];H[2][2]=N2[2];H[2][3]=0;H[2][4]=0;H[2][5]=0;

H[3][0]=N2[0];H[3][1]=N2[1];H[3][2]=N2[2];

H[3][3]=tPt2[1]*(N2[0]*bRt[0][2] + N2[1]*bRt[1][2] + N2[2]*bRt[2][2]) - tPt2[2]*(N2[0]*bRt[0][1] + N2[1]*bRt[1][1] + N2[2]*bRt[2][1]);
H[3][4]=tPt2[2]*(N2[0]*bRt[0][0] + N2[1]*bRt[1][0] + N2[2]*bRt[2][0]) - tPt2[0]*(N2[0]*bRt[0][2] + N2[1]*bRt[1][2] + N2[2]*bRt[2][2]);
H[3][5]=tPt2[0]*(N2[0]*bRt[0][1] + N2[1]*bRt[1][1] + N2[2]*bRt[2][1]) - tPt2[1]*(N2[0]*bRt[0][0] + N2[1]*bRt[1][0] + N2[2]*bRt[2][0]);

H[4][0]=1;H[4][1]=0;H[4][2]=0;H[4][3]=0;H[4][4]=0;H[4][5]=0;
H[5][0]=0;H[5][1]=1;H[5][2]=0;H[5][3]=0;H[5][4]=0;H[5][5]=0;
H[6][0]=0;H[6][1]=0;H[6][2]=1;H[6][3]=0;H[6][4]=0;H[6][5]=0;

printf("\n H=");
printfM(H);


bVt=H.pseudoInverse()*dr;

}

/*------------------------------------------------------------------------------------
 
            CurveVelocity part (b)

This function takes a polyomial curve and creates a 5 degree in time trajectory 
along this curve

    // Inputs:  Current position bMt
                CurveParameters
                CurveParameters[0]=Xt; current Xvalue of curve
                CurveParameters[1]=Xdot;
                CurveParameters[2]=Yt; current Y value of curve
                CurveParameters[3]=Ydot;
                CurveParameters[4]=theta; tangent of curve
    // Outputs: The desired frame bMtd which constrains all degrees of freedom
------------------------------------------------------------------------------------*/


void CurveVelocity(vpHomogeneousMatrix bMt,float CutDepth,vpColVector CurveParameters,vpHomogeneousMatrix& bMtd)
{

// Declarations
float Yt,Ydot,theta,Xt,Xdot,Zt;
vpMatrix H(7,6);
float N1[3];float N2[3];float N3[3];float Pd[3];
float TangentDirection[3];
vpRotationMatrix bRt;
float denom;
vpTranslationVector bPt,bPt2,tPt2;
vpColVector dr(7);

// Extract input data
Xt=CurveParameters[0];
Xdot=CurveParameters[1];
Yt=CurveParameters[2];
Ydot=CurveParameters[3];
theta=CurveParameters[4];
bMt.extract(bRt);bMt.extract(bPt);

Zt=CutDepth; // Zt t is un controlled


// Define Tangent, magic number here makes no difference to final direction
TangentDirection[0]=(Xt+0.1)-Xt;
TangentDirection[1]=(theta*((Xt+0.1)-Xt)+Yt)-Yt;
TangentDirection[2]=0.0;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;


// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=-1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

Pd[0]=Xt;
Pd[1]=Yt;
Pd[2]=Zt;

// Construct bMtd the desired frame
for(int i=0;i<3;i++)
{

    bMtd[i][0]=N1[i];
    bMtd[i][1]=TangentDirection[i];
    bMtd[i][2]=N2[i];
    bMtd[i][3]=Pd[i];
}

}



/*------------------------------------------------------------------------------------
 
            CurveEval 

This function returns the transformation matrix for a point Xt on given curve

    // Inputs:  p polynomial
                n order
                Xt position of point

    // Outputs: The desired frame bMtd which constrains all degrees of freedom
------------------------------------------------------------------------------------*/



void CurveEval(float* p,int n,vpHomogeneousMatrix bMt,vpHomogeneousMatrix& bMtd)
{

float Xt;
float Yt,theta,Zt;
float denom;
float dpdx[n-1];
polyder(p,n,dpdx);
float N1[3];float N2[3];float N3[3];float Pd[3];
float TangentDirection[3];

Xt=bMt[0][3];
Zt=bMt[2][3];
Yt=polyval(p,2,Xt);
theta=polyval(dpdx,1,Xt);

TangentDirection[0]=(Xt+0.1)-Xt;
TangentDirection[1]=(theta*((Xt+0.1)-Xt)+Yt)-Yt;
TangentDirection[2]=0.0;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;

// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=-1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

Pd[0]=Xt;
Pd[1]=Yt;
Pd[2]=Zt;

//printf("\n Evaluating(1) Curve at Xt=%f,Yt=%f,theta=%f,theta in degs=%f",Xt,Yt,theta,theta*180/PI);
// Construct bMtd the desired frame
for(int i=0;i<3;i++)
{

    bMtd[i][0]=N1[i];
    bMtd[i][1]=TangentDirection[i];
    bMtd[i][2]=N2[i];
    bMtd[i][3]=Pd[i];
}


}

void CurveEval(float* p,int n,float X,vpHomogeneousMatrix& bMtd)
{

float Xt;
float Yt,theta,Zt;
float denom;
float dpdx[n-1];
polyder(p,n,dpdx);
float N1[3];float N2[3];float N3[3];float Pd[3];
float TangentDirection[3];

Xt=X;
Yt=polyval(p,2,Xt);
Zt=0;
theta=polyval(dpdx,1,Xt);

TangentDirection[0]=(Xt+0.1)-Xt;
TangentDirection[1]=(theta*((Xt+0.1)-Xt)+Yt)-Yt;
TangentDirection[2]=0.0;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;

// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=-1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

Pd[0]=Xt;
Pd[1]=Yt;
Pd[2]=Zt;

printf("Evaluating Curve(2) at Xt=%f,Yt=%f,theta=%f,theta in degrees=%f",Xt,Yt,theta,theta*180/PI);
// Construct bMtd the desired frame
for(int i=0;i<3;i++)
{

    bMtd[i][0]=N1[i];
    bMtd[i][1]=TangentDirection[i];
    bMtd[i][2]=N2[i];
    bMtd[i][3]=Pd[i];
}


}

 /*
				FUNCTION:  ImagePointFind
//******************************************************************************************************************************************************
Inputs: Input image, filter parameters
				
Outputs: Position of points in image space

This is a simple function that takes an open cv image extracts blobs then filters blobs according to their size. 

The filtering parameters are as follows:
 TrackBars[0] a thresholding greyscale parameter
 TrackBars[1] max point size
 TrackBars[2] min point size
 TrackBars[3] We use the aspect ratio to stop misidentifications
 TrackBars[4] Cursor u
 TrackBars[5] Cursor v


Function by Philip Long
//*******************************************************************************************************************************************************


*/ 

// Attempt to functionalise the vision stuff, this is working
// Attempt to functionalise the vision stuff
void ImagePointFind(IplImage* in,IplImage* Out,int* TrackBars,vpColVector& ublob,vpColVector& vblob,int& n) 
{

    int dummy;
  	CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
	CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
	CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
	CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 
	float ARthreshold;
   //CBlobGetMinX minX = CBlobGetMinX();
   //CBlobGetMaxX maxX = CBlobGetMaxX();
   //CBlobGetMinY minY = CBlobGetMinY();
   //CBlobGetMaxY maxY = CBlobGetMaxY();

    int param1,param2,param3,param4,CursorU,CursorV;
    double MinDiff;
// Filtering options
    param1=TrackBars[0];
    param2=TrackBars[1];
    param3=TrackBars[2];
    param4=TrackBars[3];
    CursorU=TrackBars[4];
    CursorV=TrackBars[5];

    ARthreshold=(float) param4;
    IplImage* originalThr = 0;
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob BlobofInterest; // Blob used to find current informatio

    if(!originalThr) originalThr = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);


    /*---------------------------------------------------------------------- 
    --------------- FILTER 1: Greyscale threshold filter--------------------
    ------------------------------------------------------------------------ */
    cvThreshold( in, originalThr, param1, 255, CV_THRESH_BINARY );
    /*-----------------------------------------------------------------------*/ 

    /*---------------------------------------------------------------------- 
    --------------- FILTER 2: Extract the line from the image--------------
    ----------------------------------------------------------------------- */

    // find blobs in threshold image
    blobs = CBlobResult( originalThr, NULL, 255 );   	
    // Filter blobs smaller than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param3 ); 
    // Filter blobs larger than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 ); 
    // Filter blobs with a poor aspect ratio
    blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	cvMerge( originalThr, originalThr, originalThr, NULL, Out );

    MinDiff=200000; // Some high number
    n=blobs.GetNumBlobs();
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
        currentBlob = blobs.GetBlob(i);
        currentBlob->FillBlob( Out, CV_RGB(255,0,0));
        BlobofInterest=blobs.GetBlob(i);
        ublob[i]=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
        vblob[i]=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);			
        //ublob[i]=BlobofInterest.MaxX();
        //vblob[i]=maxyminx(BlobofInterest);
        //ublob[i]=BlobofInterest.MinX();
        //vblob[i]=maxyminx(BlobofInterest);
    }

    cvCircle(Out,cvPoint(CursorU,CursorV),2,CV_RGB(0, 255, 0), -1,8,0);
    cvReleaseImage( &originalThr );
}

 /*
				FUNCTION:  SquarePointFind
//******************************************************************************************************************************************************
Inputs: Input image, filter parameters
				
Outputs: An image showing only the target blob

This is a simple function that takes an open cv image extracts blobs then filters blobs according to their size. 

The filtering parameters are as follows:
 TrackBars[0] a thresholding greyscale parameter
 TrackBars[1] max point size
 TrackBars[2] min point size
 TrackBars[3] We use the aspect ratio to stop misidentifications
 TrackBars[4] Cursor u
 TrackBars[5] Cursor v


Function by Philip Long
//*******************************************************************************************************************************************************


*/ 
void SquarePointFind(IplImage* in,IplImage* Out,IplImage* Out2,int *TrackBars,float uk,float vk,vpColVector& ublob,vpColVector& vblob,int& n) 
{


  	CBlobGetMinYatMaxX minymaxx = CBlobGetMinYatMaxX();
	CBlobGetMinXatMinY minxminy = CBlobGetMinXatMinY();
	CBlobGetMaxYatMinX maxyminx = CBlobGetMaxYatMinX();
	CBlobGetMaxXatMaxY maxxmaxy = CBlobGetMaxXatMaxY(); 

    int param1,param2,param3,param4,CursorU,CursorV;
    double MinDiff;
    float ARthreshold;
// Filtering options
    param1=TrackBars[0];
    param2=TrackBars[1];
    param3=TrackBars[2];
    param4=TrackBars[3];
    CursorU=TrackBars[4];
    CursorV=TrackBars[5];
	ARthreshold=(float) param4;
	

    IplImage* originalThr = 0;
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob BlobofInterest; // Blob used to find current informatio

    if(!originalThr) originalThr = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);


    /*---------------------------------------------------------------------- 
    --------------- FILTER 1: Greyscale threshold filter--------------------
    ------------------------------------------------------------------------ */
    cvThreshold( in, originalThr, param1, 255, CV_THRESH_BINARY );
    /*-----------------------------------------------------------------------*/ 

    /*---------------------------------------------------------------------- 
    --------------- FILTER 2: Extract the line from the image--------------

    ----------------------------------------------------------------------- */

    // find blobs in threshold image
    blobs = CBlobResult( originalThr, NULL, 255 );   	
    // Filter blobs smaller than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param3 ); 
    // Filter blobs larger than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 ); 
    // Filter using the aspect ratio
    blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	cvMerge( originalThr, originalThr, originalThr, NULL, Out );

    MinDiff=200000; // Some high number
    n=blobs.GetNumBlobs();
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
        currentBlob = blobs.GetBlob(i);
        currentBlob->FillBlob( Out, CV_RGB(255,0,0));
        BlobofInterest=blobs.GetBlob(i);
        ublob[i]=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
        vblob[i]=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);			
		
    }
     float Mindist=100000;
    int SquareBlob=0;
      for(int i=0;i<fmin(n,ublob.getRows());i++)
    {

//        if(dist2D(uk,ublob[i],vk,vblob[i])<Mindist && dist2D(uk,ublob[i],vk,vblob[i])>5)
       if(dist2D(uk,ublob[i],vk,vblob[i])<Mindist && vblob[i]<vk)
        {
        Mindist=dist2D(uk,ublob[i],vk,vblob[i]);
        SquareBlob=i;
        }

    }

    if (blobs.GetNumBlobs()>0)
    {
    currentBlob = blobs.GetBlob(SquareBlob);
    currentBlob->FillBlob(Out2, CV_RGB(255,255,255));
    }
    cvCircle(Out,cvPoint(CursorU,CursorV),2,CV_RGB(0, 255, 0), -1,8,0);
    cvReleaseImage( &originalThr );

}

void SortImageVector(float uk,float vk,float& ukd,float& vkd,vpColVector& ublob,vpColVector& vblob,int& n)
{
    // Find the blob the cloeset at use this as desired point
    float Mindist=100000;

    for(int i=0;i<fmin(n,ublob.getRows());i++)
    {

//        if(dist2D(uk,ublob[i],vk,vblob[i])<Mindist && dist2D(uk,ublob[i],vk,vblob[i])>5)
       if(dist2D(uk,ublob[i],vk,vblob[i])<Mindist && vblob[i]<vk)
        {
        Mindist=dist2D(uk,ublob[i],vk,vblob[i]);
        ukd=ublob[i];vkd=vblob[i];

        }
    }
    // Now finding the blob closest to preceding one
}

 /*
				FUNCTION:  visionFramefrom3Pts
//******************************************************************************************************************************************************

Inputs: xn,yn of the points 
				
Outputs: bMv


Using the points found a line is drawn from the first point through the orders.
This line is then used as the tangent to the curve from which we can reconstruct the
frame:


    bMv[i][0]=N1[i];
    bMv[i][1]=TangentDirection[i];
    bMv[i][2]=N2[i];
    bMv[i][3]=Pd[i]; where Pd will be 3D equiv of xn1 and yn1 only using other points to
    generate tangent and normal
            

Function by Philip Long
//*******************************************************************************************************************************************************/



void visionFramefrom3Pts(float xn1,float yn1,float xn2,float yn2,float xn3,float yn3,vpHomogeneousMatrix tMcam,vpHomogeneousMatrix bMt,vpHomogeneousMatrix& bMtv,float DepthZ)
{
float denom;
float N1[3];float N2[3];float N3[3];float Pd[3];
float TangentDirection[3];
vpHomogeneousMatrix camMt;
camMt=tMcam.inverse();
vpColVector camPim1(4),camPim2(4),camPim3(4);
vpColVector bPim1(4),bPim2(4),bPim3(4);
float m2,m3,mbx;
// Get perspective points in camera frame
camPim1[0]=xn1*DepthZ; camPim1[1]=yn1*DepthZ;camPim1[2]=DepthZ;camPim1[3]=1;
camPim2[0]=xn2*DepthZ; camPim2[1]=yn2*DepthZ;camPim2[2]=DepthZ;camPim2[3]=1;
camPim3[0]=xn3*DepthZ; camPim3[1]=yn3*DepthZ;camPim3[2]=DepthZ;camPim3[3]=1;



// Step one recreate 3D position of points
bPim1=bMt*tMcam*camPim1;bPim2=bMt*tMcam*camPim2;bPim3=bMt*tMcam*camPim3;

//printf("\n Points in base frame P1=");printfM(bPim1);
//printf("\n Points in base frame P2=");printfM(bPim2);
//printf("\n Points in base frame P3=");printfM(bPim3);

// Step two find best fit line (Could be alittle complicad)
m3=( -bPim3[1]+bPim1[1] )/ ( -bPim3[0] + bPim1[0]); // Slope Line 1
m2=( bPim2[1]-bPim1[1] )/ ( bPim2[0] - bPim1[0]); // Slope Line 2
mbx=(atan(m3)+atan(m2))/2; // Slope bisector

printf("\n theta by 3 points vision=%f",mbx*180/PI);
//
// Write frame and return
TangentDirection[0]=0.1;
TangentDirection[1]=atan(mbx)*(0.1);
TangentDirection[2]=0.0;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;

// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=-1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

Pd[0]=bPim1[0];
Pd[1]=bPim1[1];
Pd[2]=0.0;

// Construct bMtd the desired frame
for(int i=0;i<3;i++)
{

    bMtv[i][0]=N1[i];
    bMtv[i][1]=TangentDirection[i];
    bMtv[i][2]=N2[i];
    bMtv[i][3]=Pd[i];
}

}




void visionFramefrom6Pts(float xn1,float yn1,float xn2,float yn2,float xn3,float yn3,float xn4,float yn4,float xn5,float yn5,float xn6,float yn6,vpHomogeneousMatrix tMcam,vpHomogeneousMatrix bMt,vpHomogeneousMatrix& bMtv,float DepthZ)
{
float denom;
float N1[3];float N2[3];float N3[3];float Pd[3];
float TangentDirection[3];
vpHomogeneousMatrix camMt;
camMt=tMcam.inverse();
vpColVector camPim1(4),camPim2(4),camPim3(4),camPim4(4),camPim5(4),camPim6(4);;
vpColVector bPim1(4),bPim2(4),bPim3(4),bPim4(4),bPim5(4),bPim6(4);
float m2,m3,m4,m5,m6,mbx;
// Get perspective points in camera frame
camPim1[0]=xn1*DepthZ; camPim1[1]=yn1*DepthZ;camPim1[2]=DepthZ;camPim1[3]=1;
camPim2[0]=xn2*DepthZ; camPim2[1]=yn2*DepthZ;camPim2[2]=DepthZ;camPim2[3]=1;
camPim3[0]=xn3*DepthZ; camPim3[1]=yn3*DepthZ;camPim3[2]=DepthZ;camPim3[3]=1;
camPim4[0]=xn4*DepthZ; camPim4[1]=yn4*DepthZ;camPim4[2]=DepthZ;camPim4[3]=1;
camPim5[0]=xn5*DepthZ; camPim5[1]=yn5*DepthZ;camPim5[2]=DepthZ;camPim5[3]=1;
camPim6[0]=xn6*DepthZ; camPim6[1]=yn6*DepthZ;camPim6[2]=DepthZ;camPim6[3]=1;


// Step one recreate 3D position of points
bPim1=bMt*tMcam*camPim1;bPim2=bMt*tMcam*camPim2;bPim3=bMt*tMcam*camPim3;

//printf("\n Points in base frame P1=");printfM(bPim1);
//printf("\n Points in base frame P2=");printfM(bPim2);
//printf("\n Points in base frame P3=");printfM(bPim3);

// Step two find best fit line (Could be alittle complicad)
m2=( bPim2[1]-bPim1[1] )/ ( bPim2[0] - bPim1[0]); // Slope Line 2
m3=( bPim3[1]-bPim1[1] )/ ( bPim3[0] - bPim1[0]); // Slope Line 3
m4=( bPim4[1]-bPim1[1] )/ ( bPim4[0] - bPim1[0]); // Slope Line 4
m5=( bPim5[1]-bPim1[1] )/ ( bPim5[0] - bPim1[0]); // Slope Line 5
m6=( bPim6[1]-bPim1[1] )/ ( bPim6[0] - bPim1[0]); // Slope Line 6
mbx=(atan(m3)+atan(m2)+atan(m4)+atan(m5)+atan(m6))/6; // Slope bisector

printf("\n theta by 6points vision=%f",mbx*180/PI);
//
// Write frame and return
TangentDirection[0]=0.1;
TangentDirection[1]=atan(mbx)*(0.1);
TangentDirection[2]=0.0;

denom=(pow((pow(TangentDirection[0],2)+pow(TangentDirection[1],2)),0.5));
TangentDirection[0]=TangentDirection[0]/denom;
TangentDirection[1]=TangentDirection[1]/denom;

// Define Normals
N1[0]=-TangentDirection[1];N1[1]=TangentDirection[0];N1[2]=0.0;
N2[0]=0.0;N2[1]=0.0;N2[2]=-1.0;
N3[0]=TangentDirection[1];N3[1]=-TangentDirection[0];N3[2]=0;

Pd[0]=bPim1[0];
Pd[1]=bPim1[1];
Pd[2]=0.0;

// Construct bMtd the desired frame
for(int i=0;i<3;i++)
{

    bMtv[i][0]=N1[i];
    bMtv[i][1]=TangentDirection[i];
    bMtv[i][2]=N2[i];
    bMtv[i][3]=Pd[i];
}

} 


/*------------------------------------------------------------------------------------

            DetectingCuttingPoint

Segments the image to leave a binary image whose white zones are the current Image position

in the case of tracking Posx and Posy are the last known positions of

------------------------------------------------------------------------------------*/
void DetectingCuttingPoint(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs)
{
    int param1,param2,param3,param4,CursorU,CursorV;
    double MinDiff;
    float ARthreshold;
    float uk,vk,ublob,vblob;
    uk=Posx;vk=Posy;
    //printf("\n Tracking or detecting blob closest to uk=%f,vk=%f",uk,vk);
// Filtering options
    param1=TrackBars[0];
    param2=TrackBars[1];
    param3=TrackBars[2];
    param4=TrackBars[3];
    CursorU=TrackBars[4];
    CursorV=TrackBars[5];
	ARthreshold=(float) param4;

    
	float Mindist=100000;
	int SquareBlob=0;


    IplImage* originalThr = 0;
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob BlobofInterest; // Blob used to find current informatio

    if(!originalThr) originalThr = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);

    /*----------------------------------------------------------------------
    --------------- FILTER 1: Greyscale threshold filter--------------------
    ------------------------------------------------------------------------ */
    cvThreshold( in, originalThr, param1, 255, CV_THRESH_BINARY );
    /*-----------------------------------------------------------------------*/

    /*----------------------------------------------------------------------
    --------------- FILTER 2: Extract the line from the image--------------
    ----------------------------------------------------------------------- */

    // find blobs in threshold image
    blobs = CBlobResult( originalThr, NULL, 255 );
    // Filter blobs smaller than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param3 );
    // Filter blobs larger than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 );
    // Filter using the aspect ratio
    blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	cvMerge( originalThr, originalThr, originalThr, NULL, Display );



	// If we have arrived here, the all blobs are within the limits, therefore its just a question of
	// location. Either the blob is close to the last one (tracking mode) or close to the standard.
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
       BlobofInterest=blobs.GetBlob(i);
       ublob=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
       vblob=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);


       if(dist2D(uk,ublob,vk,vblob)<Mindist)
        {
        Mindist=dist2D(uk,ublob,vk,vblob);
//       printf("uk=%f,vk=%f,ublob=%f,vblob=%f,Dist=%f,i=%d",uk,vk,ublob,vblob,Mindist,i);
        SquareBlob=i;
        Posx=ublob;  Posy=vblob;
        }

    }
    if (blobs.GetNumBlobs()>0)
    {
    currentBlob = blobs.GetBlob(SquareBlob);
    currentBlob->FillBlob(Out, CV_RGB(255,255,255));
//    printf("\n Blob Found at uk=%f,vk=%f",Posx,Posy);
    nblobs=blobs.GetNumBlobs();
    }
    else
    {
    nblobs=0;
    }
    cvReleaseImage( &originalThr );
}



/*------------------------------------------------------------------------------------

            Adaptive Gain:

Inputs: lambdainf, lambda0 lambdaslope, error scale
Output: lambdaApadtive

------------------------------------------------------------------------------------*/

double AdaptiveGain(double kinf,double k0,double km, double err,double xS)
{
double a=k0-kinf;
double b=km/a;
double c=kinf;
double e=fabs(err)/fabs(xS);

return (a*exp(-b*e))+c;


}

/*------------------------------------------------------------------------------------

            Adaptive Gain: return velocity

Inputs: lambdainf, lambda0 lambdaslope, error scale
Output: lambdaApadtive

------------------------------------------------------------------------------------*/

double AdaptiveGainRV(double kinf,double k0,double km, double err,double xS)
{
double a=k0-kinf;
double b=km/a;
double c=kinf;
double e=fabs(err)/fabs(xS);

return (((a*exp(-b*e))+c))*err;


}


/*------------------------------------------------------------------------------------

            DetectingCuttingPoint

Segments the image to leave a binary image whose white zones are the current Image position

in the case of tracking Posx and Posy are the last known positions of

------------------------------------------------------------------------------------*/
void DetectingCuttingPoint1(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs,int& Blob1)
{
    int param1,param2,param3,param4,CursorU,CursorV;
    double MinDiff;
    float ARthreshold;
    float uk,vk,ublob,vblob;
    uk=Posx;vk=Posy;
    //printf("\n Tracking or detecting blob closest to uk=%f,vk=%f",uk,vk);
// Filtering options
    param1=TrackBars[0];
    param2=TrackBars[1];
    param3=TrackBars[2];
    param4=TrackBars[3];
    CursorU=TrackBars[4];
    CursorV=TrackBars[5];
	ARthreshold=(float) param4;

    
	float Mindist=100000;
	int SquareBlob=0;


    IplImage* originalThr = 0;
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob BlobofInterest; // Blob used to find current informatio

    if(!originalThr) originalThr = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);

    /*----------------------------------------------------------------------
    --------------- FILTER 1: Greyscale threshold filter--------------------
    ------------------------------------------------------------------------ */
    cvThreshold( in, originalThr, param1, 255, CV_THRESH_BINARY );
    /*-----------------------------------------------------------------------*/

    /*----------------------------------------------------------------------
    --------------- FILTER 2: Extract the line from the image--------------
    ----------------------------------------------------------------------- */

    // find blobs in threshold image
    blobs = CBlobResult( originalThr, NULL, 255 );
    // Filter blobs smaller than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param3 );
    // Filter blobs larger than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 );
    // Filter using the aspect ratio
    blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	cvMerge( originalThr, originalThr, originalThr, NULL, Display );



	// If we have arrived here, the all blobs are within the limits, therefore its just a question of
	// location. Either the blob is close to the last one (tracking mode) or close to the standard.
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
       BlobofInterest=blobs.GetBlob(i);
       ublob=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
       vblob=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);


       if(dist2D(uk,ublob,vk,vblob)<Mindist)
        {
        Mindist=dist2D(uk,ublob,vk,vblob);
//       printf("uk=%f,vk=%f,ublob=%f,vblob=%f,Dist=%f,i=%d",uk,vk,ublob,vblob,Mindist,i);
        SquareBlob=i;
        Posx=ublob;  Posy=vblob;
        }

    }
    if (blobs.GetNumBlobs()>0)
    {
    currentBlob = blobs.GetBlob(SquareBlob);
    currentBlob->FillBlob(Out, CV_RGB(255,255,255));
//    printf("\n Blob Found at uk=%f,vk=%f",Posx,Posy);
    nblobs=blobs.GetNumBlobs();
    Blob1=SquareBlob;
    }
    else
    {
    nblobs=0;
    Blob1=-1;
    }
    cvReleaseImage( &originalThr );
}



/*------------------------------------------------------------------------------------

            DetectingCuttingPoint

Segments the image to leave a binary image whose white zones are the current Image position

in the case of tracking Posx and Posy are the last known positions of

------------------------------------------------------------------------------------*/
void DetectingCuttingPoint2(IplImage* in,IplImage* Out,IplImage* Display,int *TrackBars,float& Posx,float& Posy,int& nblobs,int Blob1)
{
    int param1,param2,param3,param4,CursorU,CursorV;
    double MinDiff;
    float ARthreshold;
    float uk,vk,ublob,vblob;
    uk=Posx;vk=Posy;
    //printf("\n Tracking or detecting blob closest to uk=%f,vk=%f",uk,vk);
// Filtering options
    param1=TrackBars[0];
    param2=TrackBars[1];
    param3=TrackBars[2];
    param4=TrackBars[3];
    CursorU=TrackBars[4];
    CursorV=TrackBars[5];
	ARthreshold=(float) param4;

    
	float Mindist=100000;
	int SquareBlob=0;


    IplImage* originalThr = 0;
    CBlob *currentBlob; //Pointer used to colour blobs
    CBlobResult blobs;
    CBlob BlobofInterest; // Blob used to find current informatio

    if(!originalThr) originalThr = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);

    /*----------------------------------------------------------------------
    --------------- FILTER 1: Greyscale threshold filter--------------------
    ------------------------------------------------------------------------ */
    cvThreshold( in, originalThr, param1, 255, CV_THRESH_BINARY );
    /*-----------------------------------------------------------------------*/

    /*----------------------------------------------------------------------
    --------------- FILTER 2: Extract the line from the image--------------
    ----------------------------------------------------------------------- */

    // find blobs in threshold image
    blobs = CBlobResult( originalThr, NULL, 255 );
    // Filter blobs smaller than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param3 );
    // Filter blobs larger than certain area
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, param2 );
    // Filter using the aspect ratio
    blobs.Filter( blobs, B_EXCLUDE,  CBlobGetAxisRatio(), B_LESS, ARthreshold/100 );
	cvMerge( originalThr, originalThr, originalThr, NULL, Display );



	// If we have arrived here, the all blobs are within the limits, therefore its just a question of
	// location. Either the blob is close to the last one (tracking mode) or close to the standard.
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
       BlobofInterest=blobs.GetBlob(i);
       ublob=BlobofInterest.MinX() + (( BlobofInterest.MaxX() - BlobofInterest.MinX() ) / 2.0);
       vblob=BlobofInterest.MinY() + (( BlobofInterest.MaxY() - BlobofInterest.MinY() ) / 2.0);


       if(dist2D(uk,ublob,vk,vblob)<Mindist && Blob1!=i)
        {
        Mindist=dist2D(uk,ublob,vk,vblob);
//       printf("uk=%f,vk=%f,ublob=%f,vblob=%f,Dist=%f,i=%d",uk,vk,ublob,vblob,Mindist,i);
        SquareBlob=i;
        Posx=ublob;  Posy=vblob;
        }

    }
    if (blobs.GetNumBlobs()>0)
    {
    currentBlob = blobs.GetBlob(SquareBlob);
    currentBlob->FillBlob(Out, CV_RGB(255,255,255));
  //  printf("\n Blob Found at uk=%f,vk=%f",Posx,Posy);
    nblobs=blobs.GetNumBlobs();
    }
    else
    {
    nblobs=0;
    }
    cvReleaseImage( &originalThr );
}




/*------------------------------------------------------------------------------------

    This performs the automatic update of the Jacobian Matrix
    using the rank 1 Broyden updater

Inputs: A the old estimation of the Jacobian Matrix
        dX the change in the Cartesian variables
        ds the change in the feature variables
        Gamma the weight of estimation
Outputs A the new Jacobian matrix

------------------------------------------------------------------------------------*/


void BroydenUpdater(vpMatrix& A,vpColVector& dX,vpColVector& ds,float Gamma)
{

    vpRowVector dXT(6);   
    vpColVector a1(4);
    vpMatrix B(A.getRows(),A.getCols());
    float b=0.0;


    for(int i = 0; i < 6; i++ )
    {    
    dXT[i]=dX[i];
    b=b+(dX[i]*dX[i]);
    }
    a1=ds-(A*dX);

    for(int i = 0; i < A.getRows(); i++ )
    {   
        for(int j = 0; j < A.getCols(); j++ )
        {        
        B[i][j]=a1[i]*dX[j];
        }

    }

//    printfM((B)/(b));
    A=A+(Gamma*(B)/(b));  
}





