#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

#include "estructuras.h"
#include "red_object_tracking.h"
#include "crazyflie.h"

int main(int argc, char* argv[]) {

	ThreadAttr attr;

	sem_init(&attr.mutex, 0, 1);

	pthread_t tid1, tid2;

	if (pthread_create(&tid1, NULL, startRedObjectTracking, &attr)) {

		printf("\n ERROR creating tstatic void *startRedObjectTracking(void *arg) {hread 1");
		exit(1);
	}

	startCrazyFlie(&attr);

	return 0;
}

