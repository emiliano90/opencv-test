#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "estructuras.h"
#include "camara1_detection.h"
#include "camara2_detection.h"
#include "crazyflie.h"
#include "contours.h"
#include "ball_detection.h"
#include <X11/Xlib.h>



//static void *crazyFlie(void *arg);
//static void *redObjectTracking(void *arg);

int main(int argc, char* argv[]) {
	pid_t pid;
	int status;

//	XInitThreads();
	ThreadAttr attr;
/*	attr.data.tLastPos.x = -1;
	attr.data.tLastPos.y = -1;
	attr.data.tLastPos.z = -1;
*/
//	searchContours();

	sem_init(&attr.mutex, 0, 1);

	pthread_t tid1, tid2;

//	startBallDetection();
/*
	if (pthread_create(&tid1, NULL, startCamara1, &attr)) {

		printf("\n ERROR creating tstatic void *startAlturaDetection(void *arg) {hread 1");
		exit(1);
	}
*/
/*	if (pthread_create(&tid1, NULL, startCamara2, &attr)) {

		printf("\n ERROR creating tstatic void *startRedObjectTracking(void *arg) {hread 1");
		exit(1);
	}
//*///	startRedObjectTracking(&attr);
//	wait();


	if (pthread_create(&tid1, NULL, startCamara1, &attr)) {

		printf("\n ERROR creating tstatic void *startCamara1(void *arg) {hread 1");
		exit(1);
	}
/*	if (pthread_create(&tid1, NULL, startCamara2, &attr)) {

		printf("\n ERROR creating tstatic void *startCamara2(void *arg) {hread 1");
		exit(1);
	}
*/
	startCrazyFlie(&attr);
/*	wait();
	if (pthread_create(&tid2, NULL, startCrazyFlie, &attr)) {

		printf("\n ERROR creating thread 2");
		exit(1);
	}
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

