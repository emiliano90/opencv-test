#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

#include "estructuras.h"
#include "red_object_tracking.h"
#include "crazyflie.h"





//static void *crazyFlie(void *arg);
//static void *redObjectTracking(void *arg);

int main(int argc, char* argv[]) {
//	pid_t pid;
//	int status;

	ThreadAttr attr;

	sem_init(&attr.mutex, 0, 1);

	pthread_t tid1, tid2;

	if (pthread_create(&tid1, NULL, startRedObjectTracking, &attr)) {

		printf("\n ERROR creating tstatic void *startRedObjectTracking(void *arg) {hread 1");
		exit(1);
	}

	startCrazyFlie(&attr);
/*	if (pthread_create(&tid2, NULL, startCrazyFlie, &attr)) {

		printf("\n ERROR creating thread 2");
		exit(1);
	}*/
	/*
	 if ((pid = fork()) < 0)
	 perror("fork() error");
	 else if (pid == 0) {
	 printf("Child's pid is %d and my parent's is %d\n", (int) getpid(),
	 (int) getppid());
	 mainRedObjectTracking();

	 exit(42);
	 } else {
	 puts("This is the parent.");
	 startCrazyFlie();

	 //	}
	 */
	return 0;
}

