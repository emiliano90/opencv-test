#include "Circle.h"

const int FRAMES = 8;
Circle* analizar(std::vector<Circle> estados[FRAMES]);
Circle* buscarCercano(std::vector<Circle>, Circle);
Circle toCircle(float *p);
int distancia(Circle a, Circle b);
void inicializar(CvSeq estados[FRAMES]);
int startBallDetection();

