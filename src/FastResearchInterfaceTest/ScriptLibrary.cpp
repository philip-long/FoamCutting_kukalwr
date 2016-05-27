    /* 
    ------------------------------------------------------------------------------------------------------------------------------------------------------------------		

					                Script Library 
    Contains a set of scripts that can be launched by any program that
    directly commuinicates with the FRI, therefore be CAREFUL using the scripts!


    ------------------------------------------------------------------------------------------------------------------------------------------------------------------			
    */




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
#define NUMBER_OF_JOINTS		7
using namespace std;



 /*************************************************************************************************************

    MovePoint move to a desired location in a total time using a 5th order polynomial

    *************************************************************************************************************/



void MovePoint(FastResearchInterface *FRI,vpHomogeneousMatrix Tmatrix,float TotalTime)
{
    int Moving=1;// a flag to check convergence
    float time=0.0;

    printf("\n Desired frame =\n");
    printfM(Tmatrix);
    // Define variable used from trajectory
    double alpha;
    float rt,rtdot;
    vpColVector u(3);
    vpMatrix uskew(3,3);
    vpMatrix I(3,3);
    vpMatrix A(3,3);
    vpMatrix B(3,3);
    vpMatrix C(3,3);
    vpThetaUVector uAlpha;
    float nu;
    vpRotationMatrix Rdes,R_S,Rt,Rinit,RotuAlpha,RotuAlpha_rt,RiT;
    vpTranslationVector Pt,Pinit,Pdes;
    vpHomogeneousMatrix T_traj;
    vpColVector bVt(3);
    vpColVector bOmegat(3);


    // Define FRI varibales
    float **FriJaco;  //to store the measured Jacobian from KUKA
    float cycletime;
    FriJaco = new float* [6];
    for(int i=0;i<6;i++)
    FriJaco[i] = new float[7];
    float MeasuredPose[12];
    float JointValuesInRad[7];
    float MeasuredJointValuesInRad[7];
    vpHomogeneousMatrix bTt,tTb,bTt_init;
    vpMatrix J(6,7); // Jacobian Matrix

    // Define Control variables
    vpHomogeneousMatrix S,Err;
    vpTranslationVector T_S,ErrPos;
    vpThetaUVector Sthetau,ErrRot;
     vpRotationMatrix bRt,tRb;
    vpColVector V(6),tV(6),qdot(6),qt(7);
    float dq[7];

    // END Declarations
    cycletime = FRI->GetFRICycleTime();
    FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt_init);
  	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
    for (int i = 0; i < 7; i += 1)
    {
        JointValuesInRad[i]=MeasuredJointValuesInRad[i];
    }
        


    bTt_init.extract(Pinit);    bTt_init.extract(Rinit);
    Tmatrix.extract(Pdes);    Tmatrix.extract(Rdes);

    while((FRI->IsMachineOK()) && Moving==1)
    {
    /*************************************************************************************************************

    Extract Data FRI

    *************************************************************************************************************/
        FRI->WaitForKRCTick();
        time+=cycletime;
        FRI->GetCurrentJacobianMatrix(FriJaco);
        FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame
	    FRI->GetMeasuredCartPose(MeasuredPose);
     	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
        FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt);
        tTb=bTt.inverse();
        tTb.extract(tRb);

        /*************************************************************************************************************

        Trajectory

        *************************************************************************************************************/


        if(time<TotalTime)
        {
            // Initial and final points

            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    RiT[i][j]=Rinit[j][i];
                }
            }

            // Interpolator

            rt=(    10*(pow((time/(TotalTime)),3)))    -(15*( pow((time/(TotalTime)),4)))   + (6*(pow((time/(TotalTime)),5))    );
            rtdot =( 30*pow(time,2) )/(pow((TotalTime),3)) -  (60*pow(time,3))/(pow((TotalTime),4)) +   (30*pow(time,4))/(pow((TotalTime),5));

            // Define Pt and Rt
             RotuAlpha=Rdes*RiT; // Takes the difference between the intial and final points of a section in oreintation 

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
                  T_traj[i][j]=Rt[i][j];
                }
            }
	        for(int i=0;i<3;i++)
	        {       
            	Pt[i]=Pinit[i]+rt*(Pdes[i]-Pinit[i]);
                T_traj[i][3]=Pt[i];
                bVt[i]=rtdot*(Pdes[i]-Pinit[i]);
                bOmegat[i]=(u[i]*rtdot*alpha);
	        }
            }
        else
        {
            T_traj=Tmatrix;
            for(int i=0;i<3;i++)
            {       
                bVt[i]=0.0;
                bOmegat[i]=0.0;;
            }
        }
        /*************************************************************************************************************

        Define Error between bTt_d and bTt in base frame

        *************************************************************************************************************/


        S=T_traj*bTt.inverse();
        


        S.extract(R_S);//Rotation matrix R from HomogeneousMatrix
        S.extract(T_S);//extract the deplacement vector
        Sthetau.buildFrom(R_S);//theta u representation

        /*************************************************************************************************************

        Calculate the velocity

        *************************************************************************************************************/
        for(int i=0;i<3;i++)
        {	
            V[i]=bVt[i]+(1*T_S[i]);
            V[i+3]=bOmegat[i]+(0.5*Sthetau[i]);
        } 
       
        RotateScrew(V,tV,tRb);

        qdot=J.pseudoInverse()*tV;

        for(int i=0;i<7;i++)
        {
             dq[i]=qdot[i];
             JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (dq[i]*cycletime);
        } 
        /*************************************************************************************************************

        Send data to FRI

        *************************************************************************************************************/
        Err=Tmatrix*bTt.inverse(); 
        Err.extract(ErrRot);//Rotation matrix R from HomogeneousMatrix
        Err.extract(ErrPos);//extract the deplacement vector


        FRI->SetCommandedJointPositions(JointValuesInRad);
	    /*************************************************************************************************************

       Check if we have reach the desired position with a small velocity

        *************************************************************************************************************/
            
        if  ( VectorNorm(ErrPos,3)<0.005 && VectorNorm(ErrRot,3)<0.2 && VectorNorm(bVt,3)<0.02 && VectorNorm(bOmegat,3)<0.02)
        {
            Moving=0; // escape from the loop
        }
        

        printf("time=%f",time);
        //printf("\n ===================================================");
        printf("\n ===================================================\n");
    }
}


 /*************************************************************************************************************

    MovePoint move to a desired location Tmatrixdes using exponetial decrease

        Kp,Kr are positive factors to regulate convergence usually 1 (default) is ok but it can be increased
        P stands for position
        R for orietation

    *************************************************************************************************************/
void MovePointExp(FastResearchInterface *FRI,vpHomogeneousMatrix Tmatrixdes,float Kp,float Kr)
{
    int Moving=1;// a flag to check convergence
    float time=0.0;

    printf("\n Desired frame =\n");
    printfM(Tmatrixdes);
    // Define variable used from trajectory

    vpTranslationVector bPtd,bPt,T_S;
    vpRotationMatrix bRtd,bRt,tRb,bRtdT,R_S;
    vpThetaUVector Sthetau;


    vpColVector bVt(6);



    // Define FRI varibales
    float **FriJaco;  //to store the measured Jacobian from KUKA
    float cycletime;
    FriJaco = new float* [6];
    for(int i=0;i<6;i++)
    FriJaco[i] = new float[7];
    float MeasuredPose[12];
    float JointValuesInRad[7];
    float MeasuredJointValuesInRad[7];
    vpHomogeneousMatrix bTt,tTb,bTt_init;
    vpMatrix J(6,7); // Jacobian Matrix

    // Define Control variables

    vpColVector V(6),tV(6),qdot(6),qt(7);
    float dq[7];

    // END Declarations
    cycletime = FRI->GetFRICycleTime();
    FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt_init);
  	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
    for (int i = 0; i < 7; i += 1)
    {
        JointValuesInRad[i]=MeasuredJointValuesInRad[i];
    }
        

    Tmatrixdes.extract(bPtd);   
    Tmatrixdes.extract(bRtd);  
    while((FRI->IsMachineOK()) && Moving==1)
    {
    /*************************************************************************************************************

    Extract Data FRI

    *************************************************************************************************************/
        FRI->WaitForKRCTick();
        time+=cycletime;
        FRI->GetCurrentJacobianMatrix(FriJaco);
        FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame
	    FRI->GetMeasuredCartPose(MeasuredPose);
     	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
        FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt);


        /*************************************************************************************************************

        Trajectory

        *************************************************************************************************************/


        tTb=bTt.inverse();tTb.extract(tRb);
        bTt.extract(bRt);bTt.extract(bPt);

        T_S=bPtd-bPt;

        for(int i=0;i<3;i++)
        {
        for(int j=0;j<3;j++)
        {
        bRtdT[i][j]=bRtd[j][i];
        }
        }

        R_S=bRt*bRtdT; 
        Sthetau.buildFrom(R_S);//theta u representation
        printf("Error Position=\n");printfM(T_S);
        printf("Error Orientation=\n");printfM(Sthetau);


        for(int i=0;i<3;i++)
        {	
        bVt[i]=(0.5*Kp*T_S[i]);
        bVt[i+3]=(-0.3*Kr*Sthetau[i]);
        }

        RotateScrew(bVt,tV,tRb);
        qdot=J.pseudoInverse()*tV;

        /*************************************************************************************************************

        Send to FRI

        *************************************************************************************************************/


        for(int i=0;i<7;i++)
        {
        dq[i]=qdot[i];
        JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (dq[i]*cycletime);
        } 

        FRI->SetCommandedJointPositions(JointValuesInRad);
	    /*************************************************************************************************************

       Check if we have reach the desired position with a small velocity

        *************************************************************************************************************/
            
        if  ( VectorNorm(T_S,3)<0.005 && VectorNorm(Sthetau,3)<0.05)
        {
            Moving=0; // escape from the loop
        }
        

        printf("time=%f",time);
        //printf("\n ===================================================");
        printf("\n ===================================================\n");
    }
}

 /*************************************************************************************************************

    MovePoint move to a desired location Tmatrixdes using exponetial decrease with a given accuracy

        Kp,Kr are positive factors to regulate convergence usually 1 (default) is ok but it can be increased
        P stands for position
        R for orietation
        Perr is the acceptable position error
        Rerr is the acceptable position error

    *************************************************************************************************************/
void MovePointExp(FastResearchInterface *FRI,vpHomogeneousMatrix Tmatrixdes,float Kp,float Kr,float Perr,float Rerr)
{
    int Moving=1;// a flag to check convergence
    float time=0.0;

    printf("\n Desired frame =\n");
    printfM(Tmatrixdes);
    // Define variable used from trajectory

    vpTranslationVector bPtd,bPt,T_S;
    vpRotationMatrix bRtd,bRt,tRb,bRtdT,R_S;
    vpThetaUVector Sthetau;


    vpColVector bVt(6);



    // Define FRI varibales
    float **FriJaco;  //to store the measured Jacobian from KUKA
    float cycletime;
    FriJaco = new float* [6];
    for(int i=0;i<6;i++)
    FriJaco[i] = new float[7];
    float MeasuredPose[12];
    float JointValuesInRad[7];
    float MeasuredJointValuesInRad[7];
    vpHomogeneousMatrix bTt,tTb,bTt_init;
    vpMatrix J(6,7); // Jacobian Matrix

    // Define Control variables

    vpColVector V(6),tV(6),qdot(6),qt(7);
    float dq[7];

    // END Declarations
    cycletime = FRI->GetFRICycleTime();
    FRI->GetMeasuredCartPose(MeasuredPose);
    FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt_init);
  	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
    for (int i = 0; i < 7; i += 1)
    {
        JointValuesInRad[i]=MeasuredJointValuesInRad[i];
    }
        

    Tmatrixdes.extract(bPtd);   
    Tmatrixdes.extract(bRtd);  
    while((FRI->IsMachineOK()) && Moving==1)
    {
    /*************************************************************************************************************

    Extract Data FRI

    *************************************************************************************************************/
        FRI->WaitForKRCTick();
        time+=cycletime;
        FRI->GetCurrentJacobianMatrix(FriJaco);
        FRIJaco2vpMatrix(FriJaco,J); // Get Jacobian Matrix in the KRL tool frame
	    FRI->GetMeasuredCartPose(MeasuredPose);
     	FRI->GetMeasuredJointPositions(MeasuredJointValuesInRad);
        FRICartPose2vpHomogeneousMatrix(MeasuredPose,bTt);


        /*************************************************************************************************************

        Trajectory

        *************************************************************************************************************/


        tTb=bTt.inverse();tTb.extract(tRb);
        bTt.extract(bRt);bTt.extract(bPt);

        T_S=bPtd-bPt;

        for(int i=0;i<3;i++)
        {
        for(int j=0;j<3;j++)
        {
        bRtdT[i][j]=bRtd[j][i];
        }
        }

        R_S=bRt*bRtdT; 
        Sthetau.buildFrom(R_S);//theta u representation
        printf("Error Position=\n");printfM(T_S);
        printf("Error Orientation=\n");printfM(Sthetau);


        for(int i=0;i<3;i++)
        {	
        bVt[i]=(0.5*Kp*T_S[i]);
        bVt[i+3]=(-0.3*Kr*Sthetau[i]);
        }

        RotateScrew(bVt,tV,tRb);
        qdot=J.pseudoInverse()*tV;

        /*************************************************************************************************************

        Send to FRI

        *************************************************************************************************************/


        for(int i=0;i<7;i++)
        {
        dq[i]=qdot[i];
        JointValuesInRad[i]	= MeasuredJointValuesInRad[i] + (dq[i]*cycletime);
        } 

        FRI->SetCommandedJointPositions(JointValuesInRad);
	    /*************************************************************************************************************

       Check if we have reach the desired position with a small velocity

        *************************************************************************************************************/
            
        if  ( VectorNorm(T_S,3)<Perr && VectorNorm(Sthetau,3)<Rerr)
        {
            Moving=0; // escape from the loop
        }
        

        printf("time=%f",time);
        //printf("\n ===================================================");
        printf("\n ===================================================\n");
    }
}



/*------------------------------------------------------------------------------------
 
This function loads the trajectory from the file and deals it out to a vpMatrix
------------------------------------------------------------------------------------*/
void loadtrajectory(FastResearchInterface *FRI,vpMatrix& Trajectory,int NbrofTrajectpts,const char* filename)
{
ifstream InputFile;                    // build a read-Stream
InputFile.open(filename, ios_base::in);  // open data
float Datapts; // Input data


    if (!InputFile)  // if it does not work
    {                    
        printf("Can't open Data!\n");
    }
    else // Else it has found the file
    {
        int Numberoflines=0.0; // Firstly check the number of lines

        // I know this is a ridiculous way of checking the number of lines
        // in the file, but I'm tired. 

        while(InputFile >> Datapts) 
        {
               Numberoflines=Numberoflines+1; 
        }
        if(Numberoflines/3!=NbrofTrajectpts) // Error
        {
             printf("\n \n Input trajectory does not correspond \n Either Cycle time or final time is different\n For info: \n Predicted number of datapoints=%d,\n whereas actual number=%d \n \n",NbrofTrajectpts,Numberoflines/3);
             FRI->StopRobot();	
        }
        else // If lines are in agreement, distirbute to the matrix
        {
            int row=0;
            int column=0;

            InputFile.clear();// Go back to begining of file after check
            InputFile.seekg (0, InputFile.beg);

            while(InputFile >> Datapts)
            {
               Trajectory[row][column%3]=Datapts;
               if((column%3)==2)row+=1; // Next row
                column+=1; // Newt Column

            }
        }

    }

}
