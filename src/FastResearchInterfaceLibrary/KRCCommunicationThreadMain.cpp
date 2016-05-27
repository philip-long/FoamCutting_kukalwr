//  ---------------------- Doxygen info ----------------------
//! \file KRCCommunicationThreadMain.cpp
//!
//! \brief
//! Implementation file for the thread of the class FastResearchInterface that
//! communicates with the KRC unit
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
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
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <friudp.h>
#include <friComm.h>
#include <OSAbstraction.h>


#ifdef WIN32// \ToDo Make this clean through the OSAbstraction
#include <Windows.h>	
#endif

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

// ****************************************************************
// KRCCommunicationThreadMain()
//
void* FastResearchInterface::KRCCommunicationThreadMain(void *ObjectPointer)
{
	int								SequenceCounter					=	0
								,	ResultValue						=	0;

	float							ZeroVector[LBR_MNJ];

	friUdp 							KRC(49938,"192.168.0.2");

	tFriMsrData 					LocalReadData;

	tFriCmdData						LocalCommandData;

	FastResearchInterface			*ThisObjectPtr					=	(FastResearchInterface*)ObjectPointer;
    
    struct sockaddr_in addr;	/* Address of Net F/T. */
    
    struct sockaddr_in servAddr;
	
    struct hostent *he;			/* Host entry for Net F/T. */
	
    byte request[8];			/* The request data sent to the Net F/T. */
	
    RESPONSE resp;				/* The structured response received from the Net F/T. */
	
    byte response[36];			/* The raw response data received from the Net F/T. */
	
    int i;						/* Generic loop/array index. */
	
    int err;					/* Error status of operations. */
    
    int udp_sock_sensor;
    
    int received;
    
    socklen_t sockAddrSize;
    
    int response_statut;
    
    int sent;
    
    char * AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */

	memset(ZeroVector, 0x0, LBR_MNJ * sizeof(float));
    
    sockAddrSize = sizeof(struct sockaddr_in);
    
#ifdef WIN32
	// \ToDo Make this clean through the OSAbstraction
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif	

	pthread_mutex_lock(&(ThisObjectPtr->MutexForThreadCreation));
	ThisObjectPtr->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObjectPtr->CondVarForThreadCreation));
    
    
    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

    
    /* bind local server port */
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(PORT);
    
    udp_sock_sensor = socket(AF_INET,SOCK_DGRAM,0);
    if(bind(udp_sock_sensor,(struct sockaddr *)&servAddr,sockAddrSize)<0)
	{
        std::cout<<"binding port number "<< PORT<< " failed"<<std::endl;
		exit(1);
	}
    else
    {
        char hostname[100];
		struct hostent * host;
		int i;
        
		gethostname(hostname, sizeof(hostname));
		host = gethostbyname(hostname);

        for (i=0; host->h_addr_list[i]!=0; i++)
		{
            struct in_addr addr_for_print;
            memcpy(&addr_for_print, host->h_addr_list[i], sizeof(addr_for_print));
            std::cout<<"IP "<<inet_ntoa(addr_for_print)<<" - Port "<<PORT<<std::endl;
        }
    }
    
    he = gethostbyname("192.168.0.20");
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    
    err = connect( udp_sock_sensor, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
        std::cout<<"error during force sensors connection"<<std::endl;
		exit(2);
	}
    
	for(;;)
	{
        // -----------------
        //force sensors
        
        /* Sending the request. */
        
        send( udp_sock_sensor, request, 8, 0 );
        
        /* Receiving the response. */
        recv( udp_sock_sensor, response, 36, 0 );
        resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
        resp.ft_sequence = ntohl(*(uint32*)&response[4]);
        resp.status = ntohl(*(uint32*)&response[8]);
        for( i = 0; i < 6; i++ ) {
            resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
        }

        ThisObjectPtr->Force_sensors = resp;
        //-------------
        
        
		// receive data from the KRC unit
		ResultValue	=	KRC.Recv(&LocalReadData);
        
        //std::cout<<"receive from KRC"<<std::endl;
		if (ResultValue != 0)
		{
			ThisObjectPtr->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the reception of a UDP data package.\n");
		}

		pthread_mutex_lock(&(ThisObjectPtr->MutexForControlData));

		ThisObjectPtr->NewDataFromKRCReceived	=	true;
		ThisObjectPtr->ReadData				=	LocalReadData;

		if (ThisObjectPtr->TerminateKRCCommunicationThread)
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForControlData));
			break;
		}

		LocalCommandData	=	ThisObjectPtr->CommandData;

		SequenceCounter++;
		LocalCommandData.head.sendSeqCount	=	SequenceCounter;
		LocalCommandData.head.reflSeqCount	=	LocalReadData.head.sendSeqCount;
		LocalCommandData.head.datagramId	=	FRI_DATAGRAM_ID_CMD;
		LocalCommandData.head.packetSize	=	sizeof(tFriCmdData);

		pthread_mutex_unlock(&(ThisObjectPtr->MutexForControlData));

		pthread_cond_broadcast(&(ThisObjectPtr->CondVarForDataReceptionFromKRC));
            


		// send data to KRC unit
		ResultValue						=	KRC.Send(&LocalCommandData);
		if (ResultValue != 0)
		{
			ThisObjectPtr->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the sending of a UDP data package.\n");
		}

		pthread_mutex_lock(&(ThisObjectPtr->MutexForLogging));

		if (ThisObjectPtr->LoggingIsActive)
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForLogging));
			ThisObjectPtr->DataLogger->AddEntry(		LocalReadData
			                                 	,	LocalCommandData, resp);
		}
		else
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForLogging));
		}
        
        
        
	}

	pthread_exit(NULL);
	
	return (NULL);
}


