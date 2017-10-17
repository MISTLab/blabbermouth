/*
    Simple udp server
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<math.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include "bm_pose2d.h"
//#include "server.h"
 
#define BUFLEN 3000  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data


void die(char *s)
{
    perror(s);
    exit(1);
}

void DB_unpack(char *pData);
void close(int s);

pose2d pose_opt[20];
thread_stop=0;

void server()
{

	//printf("Build the server\n");
	struct sockaddr_in si_me, si_other;

	int s, slen = sizeof(si_other) , recv_len;
	char buf[BUFLEN];

	//create a UDP socket

	socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	//bind socket to port
	bind(s , (struct sockaddr*)&si_me, sizeof(si_me) );
	



	//  printf("Waiting for data...");
	fflush(stdout);
	 
	//try to receive some data
	recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen);
	 
	//print details of the client/peer and the data received
	//printf("Received packet from %s:\n", inet_ntoa(si_other.sin_addr));
	DB_unpack(buf);       

    close(s);
    return;
}

void DB_unpack(char* pData)
{
	//printf("Begin Packet Rigid Body Data\n-------\n");

	char* ptr = pData;
	int framenumber = 0;
	memcpy(&framenumber,ptr,4);
	//printf("@unpack_dataTrans framenumber is : \n", framenumber);
	ptr += 4;

	int nMarkerSets = 0;
	memcpy(&nMarkerSets, ptr, 4);
	ptr += 4;
	//printf("@unpack_dataTransMarker Set Count : %d\n", nMarkerSets);

	// rigid bodies
	int nRigidBodies = 0;
	memcpy(&nRigidBodies, ptr, 4);
	ptr += 4;
	//printf("@unpack_dataTransRigid Body Count : %d\n", nRigidBodies);


	for (int j = 0; j < nRigidBodies; j++)
	{
		char szName[256];
		strcpy(szName, ptr);
		int nDataBytes = (int)strlen(szName) + 1;
		ptr += nDataBytes;
		//printf("@unpack_dataTransModel Name1: %s\n", szName);
	

		// rigid body pos/ori
		int ID = 0;
		ID= strtol(szName+2,NULL,10);// Decided by the name gived to the regidbody
		//memcpy(&ID, ptr, 4);
		ptr += 4;
	
		/*
		for(int i = 0; i < nDataBytes; i++){
		  printf("\n(%d) c: %c ", i, szName[i]); 
		}
		printf("---: %d\n", ID);
		*/
		
		char subbuff[2];
		memcpy( subbuff, &szName[nDataBytes - 3], 2 );
		int khepera_id = strtol(subbuff, NULL, 10);
  
		//printf(".............................. %d ", khepera_id);
		  
		float yaw = 0;
		memcpy(&yaw, ptr, 4);
		ptr += 4;

		float abx = 0;
		memcpy(&abx, ptr, 4);
		ptr += 4;

		float aby = 0;
		memcpy(&aby, ptr, 4);
		ptr += 4;

		//printf("(ID: %d): %f, %f, %f \n", ID, abx, aby, yaw);

		pose_opt[khepera_id].theta=yaw;
		pose_opt[khepera_id].x=abx; 
		pose_opt[khepera_id].y=aby;
		pose_opt[khepera_id].idr=khepera_id;

	}

}

void *thrd_func(void *arg){
	while(thread_stop==0) {
		server();
	}
}
