//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterfaceTest.cpp
//!
//! \brief
//! <b>Test application for the class FastResearchInterface</b>
//!
//! \details
//! This simple test application features a sample of how to use the Fast Research Interface
//! of the KUKA Light-Weight Robot IV. For details about the actual interface class (i.e.,
//! class FastResearchInterface), please refer to the file FastResearchInterface.h.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date November 2011
//!
//! \version 1.0
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <FastResearchInterface.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <friremote.h>




#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define SIZE_OF_TRANSFER_STRING					32

//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{




	bool					Run							=	true
						,	StartRobotCalled			=	false;

	char					c							=	0
						,	d							=	0
						,	TransferString[SIZE_OF_TRANSFER_STRING];

	unsigned int			ControlScheme				=	FastResearchInterface::JOINT_POSITION_CONTROL
						,	i							=	0
						,	LoopValue					=	0;

	int						ResultValue					=	0;

	float					FloatValues[FRI_USER_SIZE]
	     				,	TmpFloatValues[FRI_USER_SIZE]
	     				,	DesiredTorqueValues[LBR_MNJ]
	     				,	JointStiffnessValues[LBR_MNJ]
	     				,	JointDampingValues[LBR_MNJ]
	     				,	CartStiffnessValues[FRI_CART_VEC]
	     				,	CartDampingValues[FRI_CART_VEC];

	FastResearchInterface	*FRI;



	memset(TransferString		, 0x0	, SIZE_OF_TRANSFER_STRING	* sizeof(char)	);
	memset(FloatValues			, 0x0	, FRI_USER_SIZE				* sizeof(float)	);
	memset(TmpFloatValues		, 0x0	, FRI_USER_SIZE				* sizeof(float)	);
	memset(DesiredTorqueValues	, 0x0	, LBR_MNJ					* sizeof(float)	);

#ifdef WIN32
	FRI = new FastResearchInterface("E:\\Stanford\\Research\\SourceCode\\LWR_Public\\etc\\980039-FRI-Driver.init");
#endif

#ifdef __LINUX__
	fprintf(stdout, "You may need superuser permission to run this program.\n");
	fflush(stdout);
	FRI = new FastResearchInterface("/home/kuka/Documents/PhilipLong/FRILibrary/MinForceSensor/etc/980039-FRI-Driver.init");
#endif

#ifdef __MACOS__
	FRI = new FastResearchInterface("/Users/torsten/Documents/SourceCode/LWR_Public/LWR_Public/etc/980039-FRI-Driver.init");
#endif

#ifdef _NTO_
	FRI = new FastResearchInterface("/home/lwrcontrol/etc/980039-FRI-Driver2ms.init");
#endif

	for (i = 0; i < LBR_MNJ; i++)
	{
		JointStiffnessValues	[i] =	(float)10.0;
		JointDampingValues		[i]	=	(float)0.7;
	}

	for (i = 0; i < FRI_CART_VEC; i++)
	{
		CartStiffnessValues		[i]	=	(float)10.0;
		CartDampingValues		[i]	=	(float)0.7;
	}

	FRI->SetCommandedCartDamping(CartStiffnessValues);
	FRI->SetCommandedCartStiffness(CartDampingValues);
	FRI->SetCommandedJointDamping(JointDampingValues);
	FRI->SetCommandedJointStiffness(JointStiffnessValues);

//////to get a good UDP communication quality


        printf("please wait until UDP communication quality becomes perfect or \n");

	while(1)
 	{
	delay(50);
	if(FRI->GetCommunicationTimingQuality()==FRI_QUALITY_PERFECT)
  	   {
 		printf("now the UDP communication quality is perfect\n");
 		break;
	   }
	}



//////////////////

	while (Run)
	{
		printf("---------------------------------Programs provided by Stanford----------------------------\n");
		printf("Press     q  for exit this program\n");
		printf("          s  for starting the KUKA Fast Research Interface\n");
		printf("          x  for stopping the KUKA Fast Research Interface\n");
		printf("          p  for printing system information\n");
		printf("          i  for starting writing to an output file\n");
		printf("          j  for stopping writing to an output file\n");
		printf(" \n ------------------------------ Programs written by IRCCyN ------------------------------ \n");
		printf(" Press    1  Broyden Update\n");
		printf("          2  CuttingV15\n");
 		printf("          3  Cutting Comparison\n");
		printf("          4  Show various robot data (DisplayData.cpp)\n");
		printf("          5  Function Tester File (FunctionTest.cpp)\n");
		printf("          6  Calibration of Force sensor (Calibration.cpp)\n");
		printf("          7  ImageMomentCheck if wx wy are stable\n");
		printf("---------------------------------------------------------------------------------------\n\n");
		printf("Please press any key...\n");

		c	=	WaitForKBCharacter(NULL);
		
		printf("\n\n\n");

		switch (c)
		{
		case 'q':
		case 'Q':
			Run	=	false;
			break;
		case 's':
		case 'S':
			printf("Starting the robot through the FRI...\n");
			printf("Please select one of the following control strategies:\n\n");
			printf(" 1: Joint position control\n");
			printf(" 2: Cartesian impedance control\n");
			printf(" 3: Joint impedance control\n");
			printf(" a: Abort\n\n");
			d	=	0;
			while ( (d != '1') && (d != '2') && (d != '3') && (d != '9') && (d != 'a') && (d != 'A'))
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}
			if ( (d == 'a') || (d == 'A'))
			{
				printf("Control strategy remains unchanged.\n");
				break;
			}

			switch (d)
			{
			case '1':
				ControlScheme	=	FastResearchInterface::JOINT_POSITION_CONTROL;
				printf("Control strategy set to joint position control.\n");
				break;
			case '2':
				ControlScheme	=	FastResearchInterface::CART_IMPEDANCE_CONTROL;
				printf("Control strategy set to Cartesian impedance control.\n");
				break;
			case '3':
				ControlScheme	=	FastResearchInterface::JOINT_IMPEDANCE_CONTROL;
				printf("Control strategy set to joint impedance control.\n");
				break;
			}

			ResultValue	=	FRI->StartRobot(ControlScheme);

			if (ResultValue != EOK)
			{
				printf("An error occurred during starting up the robot...\n");
			}
			else
			{
				StartRobotCalled	=	true;
			}
			break;
		case 'x':
		case 'X':
			printf("Stopping the FRI...\n");
			ResultValue	=	FRI->StopRobot();
			StartRobotCalled	=	false;

			if (ResultValue != EOK)
			{
				printf("An error occurred during stopping the robot...\n");
			}
			break;
		case 'p':
		case 'P':
			printf("Printing system information...\n");
			printf("%s\n", FRI->GetCompleteRobotStateAndInformation());
			delay(200);
			break;
		case 'd':
		case 'D':
		case 'k':
		case 'K':
		case 'e':
		case 'E':
		case 'l':
		case 'L':
			if ( (c == 'd') || (c == 'D'))
			{
				printf("Changing the damping term of the joint impedance controller...\n");
			}
			else
			{
				if ( (c == 'k') || (c == 'K'))
				{
					printf("Changing the stiffness term of the joint impedance controller...\n");
				}
				else
				{
					if ( (c == 'e') || (c == 'E') )
					{
						printf("Changing the damping term of the Cartesian impedance controller...\n");
					}
					else
					{
						printf("Changing the stiffness term of the Cartesian impedance controller...\n");
					}
				}

			}

			if ( (c == 'd') || (c == 'D') || (c == 'k') || (c == 'K') )
			{
				LoopValue	=	 LBR_MNJ;
			}
			else
			{
				LoopValue	=	 FRI_CART_VEC;
			}

			printf("\nWould you like to enter one value for all vector elements (a) or for each individual one (i)?\n");

			d	=	0;
			while ( (d != 'a') && (d != 'A') && (d != 'i') && (d != 'I') )
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}

			if ( (d == 'a') || (d == 'A') )
			{
				printf("Please enter a new value for all vector elements:\n");
				printf(">");
				int tmp = scanf("%s", TransferString);	// "int tmp" to supress compiler warnings
				for (i = 0; i < LoopValue; i++)
				{
					FloatValues[i] = atof(TransferString);
				}
			}
			else
			{
				if ( (d == 'i') || (d == 'I') )
				{
					printf("Please enter new values for each vector element:\n");
					for (i = 0; i < LoopValue; i++)
					{
						printf("Element %d >", i);
						int tmp = scanf("%s", TransferString);	// "int tmp" to supress compiler warnings
						FloatValues[i] = atof(TransferString);
					}
				}
			}

			if ( (c == 'd') || (c == 'D'))
			{
				printf("New damping term of the joint impedance controller:");
			}
			else
			{
				if ( (c == 'k') || (c == 'K'))
				{
					printf("New stiffness term of the joint impedance controller:");
				}
				else
				{
					if ( (c == 'e') || (c == 'L'))
					{
						printf("New damping term of the Cartesian impedance controller:");
					}
					else
					{
						printf("New stiffness term of the Cartesian impedance controller:");
					}
				}
			}

			for (i = 0; i < LoopValue; i++)
			{
				printf("%8.3f " , FloatValues[i]);
			}

			printf("\n\nIs this correct? (y/n)\n");
			d	=	0;
			while ( (d != 'y') && (d != 'Y') && (d != 'n') && (d != 'N') )
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}

			if ( (d == 'y') || (d == 'Y') )
			{
				if ( (c == 'd') || (c == 'D'))
				{
					FRI->SetCommandedJointDamping(FloatValues);
					for (i = 0; i < LoopValue; i++)
					{
						JointDampingValues[i]	=	FloatValues[i];
					}
					printf("New joint damping values are set.\n");
				}
				else
				{
					if ( (c == 'k') || (c == 'K'))
					{
						FRI->SetCommandedJointStiffness(FloatValues);
						for (i = 0; i < LoopValue; i++)
						{
							JointStiffnessValues[i]	=	FloatValues[i];
						}
						printf("New joint stiffness values are set.\n");
					}
					else
					{
						if ( (c == 'e') || (c == 'L'))
						{
							FRI->SetCommandedCartDamping(FloatValues);
							for (i = 0; i < LoopValue; i++)
							{
								CartDampingValues[i]	=	FloatValues[i];
							}
							printf("New Cartesian damping values are set.\n");
						}
						else
						{
							FRI->SetCommandedCartStiffness(FloatValues);
							for (i = 0; i < LoopValue; i++)
							{
								CartStiffnessValues[i]	=	FloatValues[i];
							}
							printf("New Cartesian stiffness values are set.\n");
						}
					}
				}
			}
			else
			{
				printf("Values remain unchanged.\n");
			}
			break;
		
		case 'i':
		case 'I':
			printf("Starting to write an output file...\n");
			ResultValue	=	FRI->PrepareLogging("Test");
			if (ResultValue == EOK)
			{
				printf("Logging successfully prepared.\n");
			}
			else
			{
				printf( "Error at FRI->PrepareLogging(): %s\n", strerror(ResultValue));
			}

			ResultValue	=	FRI->StartLogging();
			if (ResultValue == EOK)
			{
				printf("Logging successfully started.\n");
			}
			else
			{
				printf( "Error at FRI->StartLogging(): %s\n", strerror(ResultValue));
			}
			break;

		case 'j':
		case 'J':
			printf("Stopping to write an output file...\n");
			ResultValue	=	FRI->StopLogging();
			if (ResultValue == EOK)
			{
				printf("Logging successfully stopped.\n");
			}
			else
			{
				printf( "Error at FRI->StopLogging(): %s\n", strerror(ResultValue));
			}

			ResultValue	=	FRI->WriteLoggingDataFile();
			if (ResultValue == EOK)
			{
				printf("Logging data file successfully written.\n");
			}
			else
			{
				printf( "Error at FRI->WriteLoggingDataFile(): %s\n", strerror(ResultValue));
			}
			break;

		case '1':
			printf("BroydenUpdateV1\n");
			BroydenUpdateV1(FRI);
			break;
		case '2':
			printf("CuttingV15\n");
			CuttingV15(FRI);
			break;
		case '3':
			printf("Cutting Comparison\n");
			CuttingComparison(FRI);
			break;
		case '4':
			printf("Getting Robot Data\n");
			DisplayData(FRI);
			break;
		case '5':
			printf("Function Test\n");
			FunctionTest(FRI);
			break;
		case '6':
		    printf("Calibration \n");
		    Calibration(FRI);
			break;
		case '7':
		    printf("Image Moment wx wy stability \n");
		    ImageMomentCheck(FRI);
			break;
		break;
		default:
			printf("This key is not supported yet...\n");
			break;
		}
	}

	delete FRI;

	printf("\nGood bye.\n\n");

	return(EXIT_SUCCESS);
}
