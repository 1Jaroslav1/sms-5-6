#include <stdio.h>

const int Nu = 4;
const int N = 4;
const int lambda = 0;
int D = 2;

double dU = 0;
double dUP[1]; // D-1
double s[30];

double Ku[1]; // D-1 // matlab params
double Ke; // matlab params

double y_zad = 7; // for test
double u_prev = 0; // for test

float DMC(float u, float y) {
	double e = 0;
	double Ku_dUP = 0;
	
	e = y_zad - y;
	
	for(int i = 0; i < D - 1; i++) {
		Ku_dUP += Ku[i] * dUP[i];
	}
	
	dU = Ke * e - Ku_dUP;
	
	for(int i = D; i > 0; i--){
		dUP[i] = dUP[i-1];
	}
	
	dUP[0] = dU;
	
	return u_prev + dU;
}

int main(void) {
	float ans = DMC(5, 68);
	printf("%f", ans);
	return 0;
}
