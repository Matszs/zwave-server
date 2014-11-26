/*
 * Project: 	Z-wave server
 * Door: 		Mats Otten
 * sudo gcc -o zwave-server zwave-server.c -lm -lpthread
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "lib/zwave-lib.c"

void *workingThread(void *p){
	int port = *(int *)p;

	while(1) {
		send_binary_sw_set( port, 2, 255 ); // turn node_id 2 on
		sleep(5);
		send_binary_sw_set( port, 2, 0 ); // turn node_id 2 off
		sleep(5);
	}

}

int main() {
	setvbuf (stdout, NULL, _IONBF, 0);

	int port = 0;
	int rc = 0;
	unsigned char buffer[256];
	pthread_t readThread;
	pthread_t tempThread;
	int idx = 0;
	int opt = 0;

	rc = open_port ( "/dev/ttyUSB0", &port );
	if ( rc )
		printf("Failed to open port\n");

	memset( &node_list, 0, sizeof( zwave_node_list_S ) );
	pthread_mutex_init(&list_lock, NULL);

	buffer[0] = 0x15; //NAK
	write( port, buffer, 1 );
	pthread_create(&readThread, NULL, start, (void*)&port);
	pthread_create(&tempThread, NULL, &workingThread, (void*)&port);

	buffer[0] = ZW_GET_VERSION;
	send_request( port, buffer , 1, RESP_NOT_REQ );

	buffer[0] = ZW_MEMORY_GET_ID;
	send_request( port, buffer , 1, RESP_NOT_REQ );

	buffer[0] = FUNC_ID_SERIAL_API_GET_CAPABILITIES;
	send_request( port, buffer , 1, RESP_NOT_REQ );

	buffer[0] = FUNC_ID_ZW_GET_SUC_NODE_ID;
	send_request( port, buffer , 1, RESP_NOT_REQ );

	buffer[0] = FUNC_ID_SERIAL_API_GET_INIT_DATA;
	send_request( port, buffer , 1, RESP_NOT_REQ );

	do {

		scanf( "%d", &opt );

	} while(opt != 5);

	close( port );
	return 0;
}

void ZWaveUpdate(int nodeId, int level) {
	printf("\ndevice: %d, level: %d", nodeId, level);

	//todo: place code here

	printf("\n");
}