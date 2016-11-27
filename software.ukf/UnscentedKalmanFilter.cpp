/*
********************************************************
UnscentedKalmanFilter.cpp
David Sachs
Compiles on Microsoft Visual C++ 6.0

Implements a class containing an Unscented Kalman Filter,
a Square Root Unscented Kalman Filter, and a Quaternion
Based Square Root Unscented Kalman Filter. GNU GSL is
used QR Decomposition and matrix inversion, and the
functions DCHDD and DCHUD were ported from LINPACK's
FORTRAN library, to provide cholesky factor downdating
and updating.

The state model updates angular velocity and quaternion 
orientation, and includes a gradient descent convergence 
algorithm for estimating the mean of a set of quaternions.

The observation model takes the state and predicts a set
of sensor values for a three-axis accelerometer, a 
three-axis magnetometer, and a three-axis gyroscope.

Wrapper functions are also provided for accessing the
filter as a Python module.
********************************************************
*/
#include <iostream>
#include "UnscentedKalmanFilter.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_cblas.h"
#include "gsl/gsl_blas.h"
#include "gsl/gsl_linalg.h"
#include "stdlib.h"
#include "windows.h"
#include "time.h"
#include <cmath>

#define PI 3.1415926535897931

using namespace std;

//Constructor
UKF::UKF() {}
//Destructor
UKF::~UKF() {}


/*
*********************************************************
User Accessible Filter components
*********************************************************
*/


//Filter initialization in which most of the memory used is allocated.
//This should be called once before the filtering begins.
void UKF::initFilter() {
	int i, j;
	alpha = 1;//0.01;
	beta = 2.0;
	kappa = 0;
	double processNoiseArray[STATELENGTH][STATELENGTH] =
	{
		{0.00007, 0, 0, 0, 0, 0, 0, 0, 0},//MUST BE TWEAKED!!!
		{0, 0.00007, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0.00007, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0.00012, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0.00012, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0.00012, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0.00025, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0.00025, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0.00025},

	};
	double measurementNoiseArray[MEASUREMENTLENGTH][MEASUREMENTLENGTH] =
	{

		{0.00042, 0, 0, 0, 0, 0, 0, 0, 0}, // MUST BE TWEAKED!!!
		{0, 0.00042, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0.00042, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0.00017, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0.00017, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0.00017, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0.00038, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0.00038, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0.00038},
	};
	double covarianceArray[STATELENGTH][STATELENGTH] =
	{
	{1.7, 0, 0, 0, 0, 0, 0, 0, 0},//MUST BE TWEAKED!!!
	  {0, 1.7, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 1.7, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0.1, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0.1, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0.1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0.0001, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0.0001, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0.0001},
	};

	numberOfSigmaPoints = 2*STATELENGTH+1; //Initialize UKF parameters
    lambda = (alpha*alpha)*(STATELENGTH+kappa)-STATELENGTH;
	lambda = -3;	;
    gamma = sqrt(lambda+STATELENGTH);

	//Initialize covariance weights
	covWeightZero =(lambda/(STATELENGTH+lambda))+(1-(alpha*alpha)+beta);
	covWeightOne = 1.0/(2*(STATELENGTH+lambda));
	if (covWeightOne<0)
            sqrtCovWeightOne = -sqrt(-covWeightOne);
    else
            sqrtCovWeightOne = sqrt(covWeightOne);
	if (covWeightZero < 0)
            sqrtCovWeightZero = -sqrt(-covWeightZero);
    else
            sqrtCovWeightZero = sqrt(covWeightZero);

	//Initialize all the matrix buffers
	sigBuf = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	obsSigBuf = gsl_matrix_calloc(MEASUREMENTLENGTH,numberOfSigmaPoints);
	wObsSigBuf = gsl_matrix_calloc(MEASUREMENTLENGTH,numberOfSigmaPoints);
	stateBuf = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	updStateBuf = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	wUpdStateBuf = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	mWeights = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	obsMWeights = gsl_matrix_calloc(MEASUREMENTLENGTH,numberOfSigmaPoints);
	QRBuf = gsl_matrix_calloc(3*STATELENGTH,STATELENGTH);//??????
	obsQRBuf = gsl_matrix_calloc(2*STATELENGTH + MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	processNoise = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	quatBuf = gsl_matrix_calloc(4, numberOfSigmaPoints);
	quatErrBuf = gsl_matrix_calloc(4, numberOfSigmaPoints);
	measurementNoise = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	cov = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	tempCov = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	uBuf = gsl_matrix_calloc(STATELENGTH,MEASUREMENTLENGTH);
	makeUpperTriangle = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	obsMakeUpperTriangle = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	inverted = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	invertedTrans = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	rBuf = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	obsRBuf = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	obsCov = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);
	obsCovTemp = gsl_matrix_calloc(MEASUREMENTLENGTH,MEASUREMENTLENGTH);	
	PXY = gsl_matrix_calloc(STATELENGTH, MEASUREMENTLENGTH);
	PXYTemp = gsl_matrix_calloc(STATELENGTH, MEASUREMENTLENGTH);
	kalmanGain = gsl_matrix_calloc(STATELENGTH,MEASUREMENTLENGTH);
	sigBufLong = gsl_matrix_calloc(STATELENGTH, 2*STATELENGTH);
	obsSigBufLong = gsl_matrix_calloc(MEASUREMENTLENGTH, 2*STATELENGTH);
	sigBufShort = gsl_matrix_calloc(STATELENGTH, 1);
	obsSigBufShort = gsl_matrix_calloc(MEASUREMENTLENGTH, 1);
	sigBufLongTemp = gsl_matrix_calloc(STATELENGTH, 2*STATELENGTH);
	obsSigBufLongTemp = gsl_matrix_calloc(MEASUREMENTLENGTH, 2*STATELENGTH);
	sigBufShortTemp = gsl_matrix_calloc(STATELENGTH, 1);
	obsSigBufShortTemp = gsl_matrix_calloc(MEASUREMENTLENGTH, 1);
	stateMeans = gsl_matrix_calloc(STATELENGTH,numberOfSigmaPoints);
	obsMeans = gsl_matrix_calloc(MEASUREMENTLENGTH,numberOfSigmaPoints);

	//Initialize all vector buffers
	rowVector = gsl_vector_calloc(numberOfSigmaPoints);
	state = gsl_vector_calloc(STATELENGTH);
	sqCWMean = gsl_vector_calloc(STATELENGTH);
	obsSqCWMean = gsl_vector_calloc(MEASUREMENTLENGTH);
	obsPredMean = gsl_vector_calloc(MEASUREMENTLENGTH);
	tempVec = gsl_vector_calloc(STATELENGTH);
	obsTempVec = gsl_vector_calloc(MEASUREMENTLENGTH);
	sVec = gsl_vector_calloc(STATELENGTH);
	cVec = gsl_vector_calloc(STATELENGTH);
	obsSVec = gsl_vector_calloc(MEASUREMENTLENGTH);
	obsCVec = gsl_vector_calloc(MEASUREMENTLENGTH);
	dchudTestMat = gsl_matrix_calloc(STATELENGTH,STATELENGTH);
	dchudVec = gsl_vector_calloc(STATELENGTH);
	dchddVec = gsl_vector_calloc(STATELENGTH);
	zeroVec = gsl_vector_calloc(STATELENGTH);
	measVec = gsl_vector_calloc(MEASUREMENTLENGTH);
	stateMean = gsl_vector_calloc(STATELENGTH);
	rotError = gsl_vector_calloc(3);

	//Initialize permutaion
	perm = gsl_permutation_calloc(MEASUREMENTLENGTH);	

	//Initialize quaternion buffers
	magQuat = gsl_vector_calloc(4);
	tempAngMomQuat = gsl_vector_calloc(4);
	angMomNewQuat = gsl_vector_calloc(4);
	tempGravQuat = gsl_vector_calloc(4);
	tempMagQuat = gsl_vector_calloc(4);
	gravNewQuat = gsl_vector_calloc(4);
	magNewQuat = gsl_vector_calloc(4);
	posQuatInv = gsl_vector_calloc(4);
	errQuat = gsl_vector_calloc(4);
	centAccel = gsl_vector_calloc(4);
	tempPosQuat = gsl_vector_calloc(4);
	angVQuat = gsl_vector_calloc(4);
	tempQuat = gsl_vector_calloc(4);
	posQuat = gsl_vector_calloc(4);
	gravQuat = gsl_vector_calloc(4);

	//Initialize the gravity quaternion
	gsl_vector_set(gravQuat, 2, 1.0);


	//Initialize the state vector
	gsl_vector_memcpy(stateMean, state);
	gsl_vector_set(stateMean, 3, 0.0);
	gsl_vector_set(stateMean, 4, 0.0);
	gsl_vector_set(stateMean, 5, 0.0);

	//Initialize the orientation quaternion
	gsl_vector_set(posQuat, 0, 1);//cos(ang/2));
	gsl_vector_set(posQuat, 1, 0);
	gsl_vector_set(posQuat, 2, 0);//sin(ang/2));
	gsl_vector_set(posQuat, 3, 0);

	//Transfer the initial covariance arrays to GSL matrices
	for (i=0; i<STATELENGTH; i++) {
		for (j=0; j<STATELENGTH; j++) {
			gsl_matrix_set(processNoise, i, j, processNoiseArray[i][j]);			
			gsl_matrix_set(cov, i, j, covarianceArray[i][j]);
		}

		//Make a matrix of all ones in the upper triangle
		for (j=STATELENGTH-1; j>=i; j--) {
			gsl_matrix_set(makeUpperTriangle, i, j, 1.0);
		}
	}

	//Transfer the initial covariance arrays to GSL matrices
	for (i=0; i<MEASUREMENTLENGTH; i++) {
		for (j=0; j<MEASUREMENTLENGTH; j++) {
			gsl_matrix_set(measurementNoise, i, j, measurementNoiseArray[i][j]);
		}
	//matPrint(processNoise);
	//matPrint(measurementNoise);	
		//Make a matrix of all ones in the upper triangle
		for (j=MEASUREMENTLENGTH-1; j>=i; j--) {

			gsl_matrix_set(obsMakeUpperTriangle, i, j, 1.0);
		}
	}

	//Generate UKF weights
	lambda=0;
	gsl_vector_set_all(tempVec, lambda*1.0/(STATELENGTH+lambda));
	gsl_vector_set_all(obsTempVec, lambda/(STATELENGTH+lambda));
	gsl_matrix_set_col(mWeights, 0, tempVec);
	gsl_matrix_set_col(obsMWeights, 0, obsTempVec);
	gsl_vector_set_all(tempVec, 1.0/(2*(STATELENGTH+lambda)));
	gsl_vector_set_all(obsTempVec, 1.0/(2*(STATELENGTH+lambda)));
	for (i=1; i<numberOfSigmaPoints; i++) {
		gsl_matrix_set_col(mWeights, i, tempVec);
		gsl_matrix_set_col(obsMWeights, i, obsTempVec);
	}
}


//Times the quaternion based square-root unscented Kalman filter
double UKF::timeFilter(int numTrials) {
	double freq, t, tsum=0;
	numTrials+=100;
	LARGE_INTEGER tickspersecond, tick1, tick2;
	QueryPerformanceFrequency(&tickspersecond); //Initialize timer
	freq = (double)tickspersecond.QuadPart;
	for (int i=0; i<numTrials; i++) { 
		QueryPerformanceCounter(&tick1); //Record start time
		callFilter(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); //Call filter
		QueryPerformanceCounter(&tick2); //Record end time
		t = (double)(tick2.QuadPart-tick1.QuadPart);
		if (i>100) { //Sum up the iteration times, skipping the first 100
			tsum += (t/freq);
		}
	}
	return tsum/(numTrials-100); //Return the average iteration time


}

/*
***************************************************
Unscented Kalman Filters
***************************************************
*/


//UKF
void UKF::ukf() {
	generateSigmaPoints();
	stateFunction();
	findMeanState();
	updateCovarianceState();
	generateSigmaPoints();
	observationFunction();
	findMeanObservation();
	updateCovarianceObservation();
	findKalmanGain();
	finalMean();
	finalCovariance();
}


//Square root UKF
void UKF::srukf() {
	generateSigmaPoints();
	stateFunction();
	findMeanState();
	updateCovarianceState();
	generateSigmaPoints();
	observationFunction();
	findMeanObservation();
	updateCovarianceObservation();
	findKalmanGain();
	finalMean();
	finalCovariance();
}


//Quaternion based square root UKF
void UKF::qsrukf() {
	generateSigmaPoints();
	stateFunction();
	findMeanStateQuat(); 
	updateCovarianceState(); 
	generateSigmaPoints();
	observationFunction();
	findMeanObservationQuat();
	updateCovarianceObservation();
	findKalmanGain();
	finalMean();	
	updateQuat();
	finalCovariance(); 
}


/*
**************************************************************************
These functions make it easier to interface with this code from Python
**************************************************************************
*/

/*
Allows a python script to set the measurement vector, the magnetic dip angle,
and the time since the filter was last called, and then iterate the quaternion
based square root filter.
*/
void UKF::callFilter(double measurementData1, double measurementData2, double measurementData3,double measurementData4, double measurementData5, double measurementData6, double measurementData7, double measurementData8, double measurementData9, double timeCalled, double magx, double magy, double magz) {//, double callTime) {
	gsl_vector_set(measVec, 0, measurementData1);
	gsl_vector_set(measVec, 1, measurementData2);
	gsl_vector_set(measVec, 2, measurementData3);
	gsl_vector_set(measVec, 3, measurementData4);
	gsl_vector_set(measVec, 4, measurementData5);
	gsl_vector_set(measVec, 5, measurementData6);
	gsl_vector_set(measVec, 6, measurementData7);				
	gsl_vector_set(measVec, 7, measurementData8);
	gsl_vector_set(measVec, 8, measurementData9);
	gsl_vector_set(magQuat, 1, magx);
	gsl_vector_set(magQuat, 2, magy);
	gsl_vector_set(magQuat, 3, magz);
	newTime = timeCalled;
	qsrukf();	
}


//Retrieves an element of the state vector
double UKF::getStateElement(int elementNumber) {
	
	return gsl_vector_get(stateMean, elementNumber);	
}

//Retrieves an element of the orientation quaternion
double UKF::getQuatStateElement(int elementNumber) {
	
	return gsl_vector_get(posQuat, elementNumber);
}

//Prints the noise matrices
void UKF::printNoise() {
	matPrint(measurementNoise);
	matPrint(processNoise);
}
//Sets an element of the orientation quaternion
void UKF::setQuatStateElement(int elementNumber, double value) {
	gsl_vector_set(posQuat, elementNumber, value);
}

//Sets an element of the measurement noise matrix
void UKF::setMeasurementNoise(int elementNumber, double value) {
	gsl_matrix_set(measurementNoise, elementNumber, elementNumber, value);
}

//Sets an element of the process noise matrix
void UKF::setProcessNoise(int elementNumber, double value) {
	gsl_matrix_set(processNoise, elementNumber, elementNumber, value);
}


/*
*******************************************************
Vector and matrix display utilities
*******************************************************
*/


//Display a matrix on screen (for debugging only)
void UKF::matPrint(gsl_matrix *m) {
	int i;
	for (i=0; i<m->size1; i++) {
		for (int j=0; j<m->size2; j++) {
			cout << gsl_matrix_get(m, i, j) << ' ';
		}
		cout << '\n';
	}   
	cout << '\n';
}

//Display a vector on screen (for debugging only)
void UKF::vecPrint(gsl_vector *v) {
	int i;
	for (i=0; i<v->size; i++) {
		cout << gsl_vector_get(v, i) << ' ';
	}
	cout << '\n';
}


/*
************************************************************
Basic Mathematical functions
************************************************************
*/

//Perform quaternion multiplication on qLeft and qRight, leaving the result in qLeft
void UKF::quatMult(gsl_vector *qLeft, gsl_vector *qRight)
{
	double qLeftW, qLeftX, qLeftY, qLeftZ;
	double qRightW, qRightX, qRightY, qRightZ;
	qLeftW=gsl_vector_get(qLeft, 0);
	qLeftX=gsl_vector_get(qLeft, 1);
	qLeftY=gsl_vector_get(qLeft, 2);
	qLeftZ=gsl_vector_get(qLeft, 3);
	qRightW=gsl_vector_get(qRight, 0);
	qRightX=gsl_vector_get(qRight, 1);
	qRightY=gsl_vector_get(qRight, 2);
	qRightZ=gsl_vector_get(qRight, 3);
    gsl_vector_set(tempQuat, 0, qLeftW*qRightW - qLeftX*qRightX - qLeftY*qRightY - qLeftZ*qRightZ);
	gsl_vector_set(tempQuat, 1, qLeftW*qRightX + qLeftX*qRightW + qLeftY*qRightZ - qLeftZ*qRightY);
	gsl_vector_set(tempQuat, 2, qLeftW*qRightY + qLeftY*qRightW + qLeftZ*qRightX - qLeftX*qRightZ);
	gsl_vector_set(tempQuat, 3, qLeftW*qRightZ + qLeftZ*qRightW + qLeftX*qRightY - qLeftY*qRightX);
	gsl_vector_memcpy(qLeft, tempQuat);
}

//Invert qInitial, and leave the result in qInverse
//(only works if the quaternion is normalized)
void UKF::quatInverse(gsl_vector *qInverse, gsl_vector *qInitial)
{
	gsl_vector_set(tempQuat, 0, 1);
	gsl_vector_set(tempQuat, 1, -1);
	gsl_vector_set(tempQuat, 2, -1);
	gsl_vector_set(tempQuat, 3, -1);
	gsl_vector_memcpy(qInverse, qInitial);
	gsl_vector_mul(qInverse, tempQuat);
}

//Normalize a quaternion
void UKF::quatNormalize(gsl_vector *quat) {
	double quatW, quatX, quatY, quatZ, norm;
	quatW = gsl_vector_get(quat, 0);
	quatX = gsl_vector_get(quat, 1);
	quatY = gsl_vector_get(quat, 2);
	quatZ = gsl_vector_get(quat, 3);
	norm = 1/sqrt((quatW*quatW)+(quatX*quatX)+(quatY*quatY)+(quatZ*quatZ));
	gsl_vector_scale(quat, norm);
}

//Perform a cholesky factor downdate
//Ported from the LINPACK FORTRAN function DCHDD
int UKF::dchdd(int p, gsl_vector *x, gsl_vector *c, gsl_vector *s, gsl_matrix *r) {
	  int info; //ldr,ldz,nz;
	  int i,ii,j,k;
      double alpha, norm, a; //azeta,dnrm2;
      double t, xx, scale, b; //ddot,zeta;
	  double tempVar;
	  double rvectemp[20];
	  double sVectemp[20];
	  double cVectemp[20];
	  info = 0;
	  sVectemp[0] = gsl_vector_get(x,0)/gsl_matrix_get(r, 0, 0);
	  if (p>=2) {
  		  for (j=2; j<=p; j++) {
			  for (k=0; k<p; k++) {
				  rvectemp[k]=gsl_matrix_get(r,k,j-1);
			  }
			  sVectemp[j-1] = gsl_vector_get(x, j-1) - cblas_ddot(j-1,rvectemp,1,sVectemp,1);
			  sVectemp[j-1] = sVectemp[j-1]/gsl_matrix_get(r, j-1,j-1);
		  }
	  }
	  for (k=0; k<p; k++) {
	  }
	  norm = cblas_dnrm2(p, sVectemp, 1);
	  if (norm<1.0) {
		  alpha = sqrt(1.0-norm*norm);
		  for (ii=1; ii<=p; ii++) {
			  i = p - ii + 1;
			  scale = alpha + abs(sVectemp[i-1]);
			  a = alpha/scale;
			  b=sVectemp[i-1]/scale;
			  norm=sqrt(a*a+b*b);
			  cVectemp[i-1] = a/norm;
			  sVectemp[i-1] = b/norm;
			  alpha = scale*norm;
		  }
		  for (j=1; j<=p; j++) {
			  xx = 0;
			  for (ii=1; ii<=j; ii++) {
				  i = j-ii+1;
				  t=cVectemp[i-1]*xx+sVectemp[i-1]*gsl_matrix_get(r, i-1, j-1);
				  tempVar=cVectemp[i-1]*gsl_matrix_get(r,i-1,j-1)-sVectemp[i-1]*xx;
				  gsl_matrix_set(r, i-1, j-1, tempVar);
				  xx=t;				  
			  }
		  }

	  }
	  else info = -1;
	  for (k=0; k<p; k++) {
		  gsl_vector_set(s,k,sVectemp[k]);
		  gsl_vector_set(c,k, cVectemp[k]);
	  }
	  return info;
      
}

//Perform a cholesky factor update
//Ported from the LINPACK FORTRAN function DCHUD
void UKF::dchud(int p, gsl_vector *x, gsl_vector *c, gsl_vector *s, gsl_matrix *r) {
    int j, i, jm1;
    double t, xj, rtmp, ctmp, stmp;
    for (j=1; j<=p; j++) {
		xj = gsl_vector_get(x, j-1);
        jm1 = j-1;
		if (jm1>=1) {
			for (i=1; i<=jm1; i++) {
			t = gsl_vector_get(c, (i-1))*gsl_matrix_get(r,i-1,j-1) + gsl_vector_get(s,(i-1))*xj;
            xj = gsl_vector_get(c,(i-1))*xj - gsl_vector_get(s,(i-1))*gsl_matrix_get(r,i-1,j-1);
            gsl_matrix_set(r,i-1,j-1,t);
			}
		}
		rtmp = gsl_matrix_get(r, j-1, j-1);
		ctmp = gsl_vector_get(c, j-1);
		stmp = gsl_vector_get(s, j-1);
		cblas_drotg(&rtmp,&xj,&ctmp,&stmp);
		gsl_matrix_set(r, j-1, j-1, rtmp);
		gsl_vector_set(c, j-1, ctmp);
		gsl_vector_set(s, j-1, stmp);
	}
}


/*
*******************************************************
Models used in filter
*******************************************************
*/


//Update the state vector in a quaterion square root UKF
void UKF::stateFunction() {
	int i;
	double angV1, angV2, angV3, angMag, axis1, axis2, axis3, deltaAngle;
	double angA1, angA2, angA3;
	double angE1, angE2, angE3;
	//Update all state elements as random walks (for now)
	gsl_matrix_memcpy(updStateBuf, stateBuf); 

	//Iterate over all sigma points
	for (i=0; i<numberOfSigmaPoints; i++) {
		angA1 = gsl_matrix_get(stateBuf, 6, i);
		angA2 = gsl_matrix_get(stateBuf, 7, i);
		angA3 = gsl_matrix_get(stateBuf, 8, i);

		//Get angular velocity
		angV1 = gsl_matrix_get(stateBuf, 3, i);
		angV2 = gsl_matrix_get(stateBuf, 4, i);
		angV3 = gsl_matrix_get(stateBuf, 5, i);

		//Combine angular velocity with acceleration
		angV1=angV1+angA1*newTime;
		angV2=angV2+angA2*newTime;
		angV3=angV3+angA3*newTime;

		//Create a perturbation angle
		angE1=angV1*newTime+0.5*angA1*newTime*newTime;
		angE2=angV2*newTime+0.5*angA2*newTime*newTime;
		angE3=angV3*newTime+0.5*angA3*newTime*newTime;

		//Separate into magnitude and axis of rotation of a delta vector
		angMag = sqrt((angE1*angE1)+(angE2*angE2)+(angE3*angE3));
		deltaAngle = angMag;
		if (angMag!=0) {
			axis1 = angE1/angMag;
			axis2 = angE2/angMag;
			axis3 = angE3/angMag;
		}
		else {
			axis1 = 0;
			axis2 = 0;
			axis3 = 0;
			deltaAngle=0;
		}

		//Translate into a delta quaternion
		gsl_vector_set(angVQuat, 0, cos(deltaAngle/2));
		gsl_vector_set(angVQuat, 1, sin(deltaAngle/2)*axis1);
		gsl_vector_set(angVQuat, 2, sin(deltaAngle/2)*axis2);
		gsl_vector_set(angVQuat, 3, sin(deltaAngle/2)*axis3);
		
		//Get the error vector
		angV1 = gsl_matrix_get(stateBuf, 0, i);
		angV2 = gsl_matrix_get(stateBuf, 1, i);
		angV3 = gsl_matrix_get(stateBuf, 2, i);

		//Separate into magnitude and axis of rotation
		angMag = sqrt((angV1*angV1)+(angV2*angV2)+(angV3*angV3));
		deltaAngle = angMag;//*newTime;
		if (angMag!=0) {
			axis1 = angV1/angMag;
			axis2 = angV2/angMag;
			axis3 = angV3/angMag;
		}
		else {
			axis1 = 0;
			axis2 = 0;
			axis3 = 0;
			deltaAngle=0;
		}

		//Form quaternion
		gsl_vector_set(errQuat, 0, cos(deltaAngle/2));
		gsl_vector_set(errQuat, 1, sin(deltaAngle/2)*axis1);
		gsl_vector_set(errQuat, 2, sin(deltaAngle/2)*axis2);
		gsl_vector_set(errQuat, 3, sin(deltaAngle/2)*axis3);

		//Update position quaternion with error quaternion and change
		//due to angular velocity
		quatMult(angVQuat, errQuat);
		gsl_vector_memcpy(tempQuat, posQuat);
		quatMult(tempQuat, angVQuat);
		gsl_matrix_set_col(quatBuf, i, tempQuat);
	}
}



//Update the observation vector for a quaternion based square root UKF
void UKF::observationFunction() {
	double errorangle;
	double cent, norm;
	int i;

	//Back up posQuat
	gsl_vector_memcpy(tempPosQuat, posQuat);
	
	//Transfer angular velocity directly to the observation buffer
	for (i=3; i<6; i++) {
		gsl_matrix_get_row(rowVector, stateBuf, i);
		gsl_matrix_set_row(obsSigBuf, i+3, rowVector);
	}

	//Iterate over all sigma points
	for (i=0; i<numberOfSigmaPoints; i++) {

		//Back up reference vectors
		gsl_vector_memcpy(tempPosQuat, posQuat);
		gsl_vector_memcpy(tempMagQuat, magQuat);
		gsl_vector_memcpy(tempGravQuat, gravQuat);

		//Get error vector
		gsl_vector_set(rotError, 0, gsl_matrix_get(stateBuf, 0, i));
		gsl_vector_set(rotError, 1, gsl_matrix_get(stateBuf, 1, i));
		gsl_vector_set(rotError, 2, gsl_matrix_get(stateBuf, 2, i));

		//Separate into magnitude and axis of rotation
		errorangle = gsl_blas_dnrm2(rotError);
		if (errorangle!=0) {
			gsl_vector_scale(rotError, 1.0/errorangle);
		}

		//Assemble error quaternion
		gsl_vector_set(errQuat, 0, cos(errorangle/2));
		gsl_vector_set(errQuat, 1, gsl_vector_get(rotError, 0)*sin(errorangle/2));
		gsl_vector_set(errQuat, 2, gsl_vector_get(rotError, 1)*sin(errorangle/2));
		gsl_vector_set(errQuat, 3, gsl_vector_get(rotError, 2)*sin(errorangle/2));

		//Adjust orientation quaternion
		quatMult(tempPosQuat, errQuat);
		norm = gsl_blas_dnrm2(tempPosQuat);
		if (norm!=0) {
			gsl_vector_scale(tempPosQuat, 1.0/norm);
		}

		//Get inverse of orientation quaternion
		quatInverse(posQuatInv, tempPosQuat);

		//calculate centripetal acceleration
		gsl_vector_set(centAccel, 1, gsl_matrix_get(stateBuf, 3, i));
		gsl_vector_set(centAccel, 2, gsl_matrix_get(stateBuf, 4, i));
		gsl_vector_set(centAccel, 3, gsl_matrix_get(stateBuf, 5, i));
		cent = gsl_blas_dnrm2(centAccel);
		if (cent!=0) {
			gsl_vector_scale(centAccel, 1.0/cent);
		}
		cent = cent*cent;
		cent = cent/500.0;
		cent = cent/9.8;
		gsl_vector_scale(centAccel, cent);
		//cent = gsl_vector_get(centAccel, 2);
		gsl_vector_set(centAccel, 2, cent/50.0);
		//gsl_vector_scale(centAccel, 0);

		
		//Rotate gravity vector with orientation quaternion
		//to predict accelerometer values
		quatMult(tempGravQuat, posQuatInv);
		gsl_vector_memcpy(gravNewQuat, tempPosQuat);
		quatMult(gravNewQuat, tempGravQuat);
		//cout << "centaccel\n";
		//vecPrint(centAccel);
		//gsl_vector_add(gravNewQuat, centAccel);
		gsl_matrix_set(obsSigBuf, 0, i, gsl_vector_get(gravNewQuat, 1));
		gsl_matrix_set(obsSigBuf, 1, i, gsl_vector_get(gravNewQuat, 2));
		gsl_matrix_set(obsSigBuf, 2, i, gsl_vector_get(gravNewQuat, 3));

		//Rotate magnetic field vector with orientation quaternion
		//to predict magnetometer values
		quatMult(tempMagQuat, posQuatInv);
		gsl_vector_memcpy(magNewQuat, tempPosQuat);
		quatMult(magNewQuat, tempMagQuat);
		gsl_matrix_set(obsSigBuf, 3, i, gsl_vector_get(magNewQuat, 1));
		gsl_matrix_set(obsSigBuf, 4, i, gsl_vector_get(magNewQuat, 2));
		gsl_matrix_set(obsSigBuf, 5, i, gsl_vector_get(magNewQuat, 3));

		//Rotate angular momentum vector with orientation quaternion
		//to predict gyroscope values
		gsl_vector_set(tempAngMomQuat, 0, 0);
		gsl_vector_set(tempAngMomQuat, 1, gsl_matrix_get(stateBuf, 3, i));
		gsl_vector_set(tempAngMomQuat, 2, gsl_matrix_get(stateBuf, 4, i));
		gsl_vector_set(tempAngMomQuat, 3, gsl_matrix_get(stateBuf, 5, i));
		quatMult(tempAngMomQuat, posQuatInv);
		gsl_vector_memcpy(angMomNewQuat, tempPosQuat);
		quatMult(angMomNewQuat, tempAngMomQuat);
		
		gsl_matrix_set(obsSigBuf, 6, i, gsl_vector_get(angMomNewQuat, 1));
		gsl_matrix_set(obsSigBuf, 7, i, gsl_vector_get(angMomNewQuat, 2));
		gsl_matrix_set(obsSigBuf, 8, i, gsl_vector_get(angMomNewQuat, 3));
	}
}


//Update the orientation quaternion
void UKF::updateQuat() {
	double errorangle, norm;

	//Get the final estimated error
	gsl_vector_set(rotError, 0, gsl_vector_get(stateMean, 0));
	gsl_vector_set(rotError, 1, gsl_vector_get(stateMean, 1));
	gsl_vector_set(rotError, 2, gsl_vector_get(stateMean, 2));
	errorangle = gsl_blas_dnrm2(rotError);
	//while (errorangle>(2*PI) && (errorangle<100)) {
	//	errorangle=errorangle-(2*PI);
	//}
	//double errnorm;
	//errnorm = gsl_blas_dnrm2(errQuat);
	//cout << "errorangle " << errorangle << "\n";
	//errorangle=errorangle*-1;
	if (errorangle!=0) {
		gsl_vector_scale(rotError, 1.0/errorangle);
	}

	//Assemble quaternion
	gsl_vector_set(errQuat, 0, cos(errorangle/2));
	gsl_vector_set(errQuat, 1, gsl_vector_get(rotError, 0)*sin(errorangle/2));
	gsl_vector_set(errQuat, 2, gsl_vector_get(rotError, 1)*sin(errorangle/2));
	gsl_vector_set(errQuat, 3, gsl_vector_get(rotError, 2)*sin(errorangle/2));
	double errnorm;
	errnorm = gsl_blas_dnrm2(errQuat);
	//cout << "errnorm " << errnorm << "\n";
	//Rotate old quaternion to form best guess of new quaternion
	quatMult(posQuat, errQuat);
	norm = gsl_blas_dnrm2(posQuat);
	//cout << "norm " << norm << "\n";
	if (norm!=0) {
		gsl_vector_scale(posQuat, 1.0/norm);
	}
	gsl_vector_set(stateMean, 0, 0);
	gsl_vector_set(stateMean, 1, 0);
	gsl_vector_set(stateMean, 2, 0);
}


//Generate a set of sigma points from a state vector
void UKF::generateSigmaPoints() {
	int i;

	//Arrange state vectors in columns
	for (i=0; i<numberOfSigmaPoints; i++) {
		gsl_matrix_set_col(stateBuf, i, stateMean);
	}
	gsl_matrix_set_col(sigBuf, 0, zeroVec);

	//Arrange covariance in sigBuf
	for (i=0; i<STATELENGTH; i++) {
		gsl_matrix_get_col(tempVec, cov, i);
		gsl_matrix_set_col(sigBuf, i+1, tempVec);
		gsl_matrix_get_col(tempVec, cov, i);
		gsl_vector_scale(tempVec, -1);
		gsl_matrix_set_col(sigBuf, i+STATELENGTH+1, tempVec);
	}

	//Introduce weights
	gsl_matrix_scale(sigBuf, gamma);

	//Combine buffers to finish making sigma points
	gsl_matrix_add(sigBuf, stateBuf);
	gsl_matrix_memcpy(stateBuf, sigBuf);
}


//Calculate the mean of a set of sigma points
void UKF::findMeanState() {
	int i;
	gsl_matrix_memcpy(wUpdStateBuf, updStateBuf);

	//Weight sigma point vectors
	gsl_matrix_mul_elements(wUpdStateBuf, mWeights); 
	gsl_matrix_get_col(stateMean, wUpdStateBuf, 0);

	//Sum up points
	for (i=1; i<numberOfSigmaPoints; i++) {
		gsl_matrix_get_col(tempVec, wUpdStateBuf, i);
		gsl_vector_add(stateMean, tempVec);
	}
}


//Calculate the mean of a set of sigma point quaternions
void UKF::findMeanStateQuat() {
	int i, j;
	double angV1, angV2, angV3, angMag, axis1, axis2, axis3, deltaAngle, errorangle, norm;
	
	gsl_vector_memcpy(tempPosQuat, posQuat);
	gsl_matrix_memcpy(quatErrBuf, quatBuf);
	quatInverse(posQuatInv, tempPosQuat);

	//Iterate over sigma points
	for (i=0; i<numberOfSigmaPoints; i++) {
		gsl_matrix_get_col(tempQuat, quatErrBuf, i);
		quatMult(tempQuat, posQuatInv);
		quatNormalize(tempQuat);
		double errang;
		errang = gsl_vector_get(tempQuat, 0);
		errorangle = 2*acos(errang);
		gsl_vector_set(tempQuat, 0, 0);
		norm = gsl_blas_dnrm2(tempQuat);
		if (norm!=0) {
			gsl_vector_scale(tempQuat, 1.0/norm);
		}
		gsl_matrix_set(updStateBuf, 0, i, gsl_vector_get(tempQuat, 1)*errorangle);
		gsl_matrix_set(updStateBuf, 1, i, gsl_vector_get(tempQuat, 2)*errorangle);
		gsl_matrix_set(updStateBuf, 2, i, gsl_vector_get(tempQuat, 3)*errorangle);
	}
	gsl_matrix_memcpy(wUpdStateBuf, updStateBuf);

	//Alter weights for quaternion use
	for (i=0; i<numberOfSigmaPoints; i++) {
		for (j=0; j<3; j++) {
			gsl_matrix_set(mWeights, j, i, 1.0/numberOfSigmaPoints);
		}
	}

	//Weight error vectors
	gsl_matrix_mul_elements(wUpdStateBuf, mWeights);
	gsl_matrix_get_col(stateMean, wUpdStateBuf, 0);

	//Sum over vectors and angular velocity sigma points
	for (i=1; i<numberOfSigmaPoints; i++) {
		gsl_matrix_get_col(tempVec, wUpdStateBuf, i);
		gsl_vector_add(stateMean, tempVec);
	}

	//Get error vector from stateMean
	angV1 = gsl_vector_get(stateMean, 0);
	angV2 = gsl_vector_get(stateMean, 1);
	angV3 = gsl_vector_get(stateMean, 2);
	angMag = sqrt((angV1*angV1)+(angV2*angV2)+(angV3*angV3));
	deltaAngle = angMag;
	if (angMag!=0) {
		axis1 = angV1/angMag;
		axis2 = angV2/angMag;
		axis3 = angV3/angMag;
	}
	else {
		axis1 = 0;
		axis2 = 0;
		axis3 = 0;
		deltaAngle = 0;
	}

	//Form quaternion
	gsl_vector_set(tempQuat, 0, cos(deltaAngle/2));
	gsl_vector_set(tempQuat, 1, sin(deltaAngle/2)*axis1);
	gsl_vector_set(tempQuat, 2, sin(deltaAngle/2)*axis2);
	gsl_vector_set(tempQuat, 3, sin(deltaAngle/2)*axis3);
	quatInverse(tempPosQuat, tempQuat);
	quatMult(posQuat, tempPosQuat);	
	quatInverse(tempPosQuat, posQuat);
	gsl_vector_memcpy(posQuat, tempPosQuat);
	norm = gsl_blas_dnrm2(posQuat);
	if (norm!=0) {
		gsl_vector_scale(posQuat, 1.0/norm);
	}
	quatInverse(posQuatInv, posQuat);

	//Iterating over gradient descent algorithm
	for (j=0; j<10; j++) {
		gsl_matrix_memcpy(quatErrBuf, quatBuf);

		//Iterating over sigma pionts
		for (i=0; i<numberOfSigmaPoints; i++) {

			//Remove orientation quaternion from perturbed quaternions
			gsl_matrix_get_col(tempQuat, quatErrBuf, i);
			quatMult(tempQuat, posQuatInv);
			quatNormalize(tempQuat);

			//Form error vectors from quaternions
			errorangle = 2*acos(gsl_vector_get(tempQuat, 0));
			gsl_vector_set(tempQuat, 0, 0);
			norm = gsl_blas_dnrm2(tempQuat);

			if (norm!=0) {
				gsl_vector_scale(tempQuat, 1.0/norm);
			}

			gsl_vector_scale(tempQuat, errorangle);
			gsl_matrix_set_col(quatErrBuf, i, tempQuat);
		}
		
		//Sum error vectors
		gsl_matrix_scale(quatErrBuf, 1.0/numberOfSigmaPoints);
		gsl_matrix_get_col(tempPosQuat, quatErrBuf, 0);
		for (i=1; i<numberOfSigmaPoints; i++) {
			gsl_matrix_get_col(tempQuat, quatErrBuf, i);
			gsl_vector_add(tempPosQuat, tempQuat);
		}

		//Get error quaternion from final error vector
		angV1 = gsl_vector_get(tempPosQuat, 1);
		angV2 = gsl_vector_get(tempPosQuat, 2);
		angV3 = gsl_vector_get(tempPosQuat, 3);
		angMag = sqrt((angV1*angV1)+(angV2*angV2)+(angV3*angV3));
		deltaAngle = angMag;//*newTime;
		if (angMag!=0) {
			axis1 = angV1/angMag;
			axis2 = angV2/angMag;
			axis3 = angV3/angMag;
		}
		else {
			axis1 = 0;
			axis2 = 0;
		axis3 = 0;
		deltaAngle = 0;
		}
		gsl_vector_set(tempQuat, 0, cos(deltaAngle/2));
		gsl_vector_set(tempQuat, 1, sin(deltaAngle/2)*axis1);
		gsl_vector_set(tempQuat, 2, sin(deltaAngle/2)*axis2);
		gsl_vector_set(tempQuat, 3, sin(deltaAngle/2)*axis3);

		//Adjust the mean, and iterative the algorithm again
		quatMult(tempQuat, posQuat);
		gsl_vector_memcpy(posQuat, tempQuat);
		quatInverse(posQuatInv, posQuat);
	}

	//Set updated state buffer
	for (i=0; i<3; i++) {
		gsl_matrix_get_row(rowVector, quatErrBuf, i+1);
		gsl_matrix_set_row(updStateBuf, i, rowVector);
	}

	//Set state mean
	gsl_vector_set(stateMean, 0, gsl_vector_get(tempPosQuat,1));
	gsl_vector_set(stateMean, 1, gsl_vector_get(tempPosQuat,2));
	gsl_vector_set(stateMean, 2, gsl_vector_get(tempPosQuat,3));
	gsl_matrix_set_col(updStateBuf, 0, stateMean);
}


//Find the mean observation vector
void UKF::findMeanObservation() {
	int i;
	gsl_matrix_memcpy(wObsSigBuf, obsSigBuf);

	//Weight observation buffer
	gsl_matrix_mul_elements(wObsSigBuf, obsMWeights);
	gsl_matrix_get_col(obsPredMean, wObsSigBuf, 0);

	//Sum over observation sigma points
	for (i=1; i<numberOfSigmaPoints; i++) {
		gsl_matrix_get_col(obsTempVec, wObsSigBuf, i);
		gsl_vector_add(obsPredMean, obsTempVec);
	}
}


//Find the mean observation vector using quaternions (under construction)
void UKF::findMeanObservationQuat() {
	int i;
	gsl_matrix_memcpy(wObsSigBuf, obsSigBuf);

	//Weight observation buffer
	gsl_matrix_mul_elements(wObsSigBuf, obsMWeights);
	gsl_matrix_get_col(obsPredMean, wObsSigBuf, 0);

	//Sum over observation sigma points
	for (i=1; i<numberOfSigmaPoints; i++) {
		gsl_matrix_get_col(obsTempVec, wObsSigBuf, i);
		gsl_vector_add(obsPredMean, obsTempVec);
	}

	double magmag, magx, magy, magz, accx, accy, accz;

	//Retrieve accelerometer and magnetic field sensor values
	accx=gsl_vector_get(obsPredMean, 0);
	accy=gsl_vector_get(obsPredMean, 1);
	accz=gsl_vector_get(obsPredMean, 2);
	magx=gsl_vector_get(obsPredMean, 3);
	magy=gsl_vector_get(obsPredMean, 4);
	magz=gsl_vector_get(obsPredMean, 5);

	//Normalize magnetic field sensors values
	magmag=1;//sqrt(magx*magx+magy*magy+magz*magz);
	gsl_vector_set(obsPredMean, 0, accx);
	gsl_vector_set(obsPredMean, 1, accy);
	gsl_vector_set(obsPredMean, 2, accz);
	gsl_vector_set(obsPredMean, 3, magx/magmag);
	gsl_vector_set(obsPredMean, 4, magy/magmag);
	gsl_vector_set(obsPredMean, 5, magz/magmag);

}


//Update the covariance of the state vector
void UKF::updateCovarianceState() {
	int i, dchddfail;

	//Prepare for QR Decomposition
	for (i=0; i<(2*STATELENGTH); i++) {
		gsl_matrix_get_col(tempVec, updStateBuf, i+1);
		gsl_vector_sub(tempVec, stateMean);
		gsl_vector_scale(tempVec, sqrtCovWeightOne);
		gsl_matrix_set_row(QRBuf, i, tempVec);
	}
	for (i=0; i<STATELENGTH; i++) {
		gsl_matrix_get_col(tempVec, processNoise, i);
		gsl_matrix_set_row(QRBuf, i+STATELENGTH*2, tempVec);
	}

	//Perform QR Decomposition
	gsl_linalg_QR_decomp(QRBuf, tempVec);
	for (i=0; i<STATELENGTH; i++) {
		gsl_matrix_get_row(tempVec, QRBuf, i);
		gsl_matrix_set_row(cov, i, tempVec);
	}

	//Prepare for cholesky factor updating/downdating
	gsl_matrix_mul_elements(cov, makeUpperTriangle);
	gsl_vector_memcpy(sqCWMean, stateMean);
	gsl_matrix_get_col(tempVec, updStateBuf, 0);
	gsl_vector_sub(tempVec, sqCWMean);
	gsl_vector_memcpy(sqCWMean, tempVec);
	gsl_vector_scale(sqCWMean, sqrtCovWeightZero);

	//Perform cholesky factor updating/downdating
	if (covWeightZero>=0) dchud(STATELENGTH, sqCWMean, cVec, sVec, cov);
	else {
		dchddfail = dchdd(STATELENGTH,stateMean, cVec, sVec, cov);
		cout << "downdating\n";
		if (dchddfail != 0) cout << "UPDATE FAILED!\n";
	}
}


void UKF::updateCovarianceObservation() {
	int i, dchddfail;

	//Prepare for QR Decomposition
	for (i=0; i<(2*STATELENGTH); i++) {
		gsl_matrix_get_col(obsTempVec, obsSigBuf, i+1);
		gsl_vector_sub(obsTempVec, obsPredMean);
		gsl_vector_scale(obsTempVec, sqrtCovWeightOne);
		gsl_matrix_set_row(obsQRBuf, i, obsTempVec);
	}
	for (i=0; i<MEASUREMENTLENGTH; i++) {
		gsl_matrix_get_col(obsTempVec, measurementNoise, i);
		gsl_matrix_set_row(obsQRBuf, i+(2*STATELENGTH), obsTempVec);
	}

	//Perform QR Decomposition
	gsl_linalg_QR_decomp(obsQRBuf, obsTempVec);
	
	for (i=0; i<MEASUREMENTLENGTH; i++) {
		gsl_matrix_get_row(obsTempVec, obsQRBuf, i);
		gsl_matrix_set_row(obsCov, i, obsTempVec);
	}
	//Prepare for cholesky factor updating/downdating
	
	gsl_matrix_mul_elements(obsCov, obsMakeUpperTriangle);
	gsl_vector_memcpy(obsSqCWMean, obsPredMean);
	gsl_matrix_get_col(obsTempVec, obsSigBuf, 0);
	gsl_vector_sub(obsTempVec, obsSqCWMean);
	gsl_vector_memcpy(obsSqCWMean, obsTempVec);
	gsl_vector_scale(obsSqCWMean, sqrtCovWeightZero);

	//Perform cholesky factor updating/downdating
	if (covWeightZero>=0) dchud(MEASUREMENTLENGTH, obsSqCWMean, obsCVec, obsSVec, obsCov);
	else {
		dchddfail = dchdd(MEASUREMENTLENGTH,obsSqCWMean, obsCVec, obsSVec, obsCov);
		if (dchddfail != 0) cout << "UPDATE FAILED!\n";
	}
}


//Calculate the Kalman gain
void UKF::findKalmanGain() {
	int i;

	//Prepare state and observation buffers
	for (i=0; i<numberOfSigmaPoints; i++) {
		gsl_matrix_set_col(stateMeans, i, stateMean);
		gsl_matrix_set_col(obsMeans, i, obsPredMean);
	}

	//Prepare error buffers
	gsl_matrix_sub(sigBuf, stateMeans);
	gsl_matrix_sub(obsSigBuf, obsMeans);
	gsl_matrix_get_col(tempVec, sigBuf, 0);
	gsl_matrix_set_col(sigBufShort, 0, tempVec);
	gsl_matrix_get_col(obsTempVec, obsSigBuf, 0);
	gsl_matrix_set_col(obsSigBufShort, 0, obsTempVec);

	for (i=0; i<2*STATELENGTH; i++) {
		gsl_matrix_get_col(tempVec, sigBuf, i+1);
		gsl_matrix_set_col(sigBufLong, i, tempVec);
	}
	for (i=0; i<2*STATELENGTH; i++) {
		gsl_matrix_get_col(obsTempVec, obsSigBuf, i+1);
		gsl_matrix_set_col(obsSigBufLong, i, obsTempVec);
	}

	//Form intermediate covariance matrices
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, sigBufShort, obsSigBufShort, 0, PXYTemp);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, sigBufLong, obsSigBufLong, 0, PXY);
	gsl_matrix_scale(PXY, covWeightOne);
	gsl_matrix_scale(PXYTemp, covWeightZero);
	gsl_matrix_add(PXY, PXYTemp);

	//Combine covariance matrices to form Kalman gain
	gsl_matrix_memcpy(obsRBuf, obsCov);
	gsl_matrix_transpose(obsRBuf);

	//Solve matrix right divide using back-substitution
	gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasNonUnit, 1, obsCov, PXY);
	gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasNonUnit, 1, obsRBuf, PXY);
	gsl_matrix_memcpy(kalmanGain, PXY);

}


//Subtract inovation, scale with Kalman gain, and form the final mean
void UKF::finalMean() {
	gsl_vector_memcpy(obsTempVec, measVec);
	//gsl_vector_sub(measVec, obsPredMean);
	gsl_vector_sub(obsTempVec, obsPredMean);
	//vecPrint(obsTempVec);
	//gsl_blas_dgemv(CblasNoTrans, 1, kalmanGain, measVec, 0, tempVec);
	gsl_blas_dgemv(CblasNoTrans, 1, kalmanGain, obsTempVec, 0, tempVec);
	gsl_vector_add(stateMean, tempVec);
	
}


//Perform repeated cholesky factor downdating and calculate the final covariance
void UKF::finalCovariance() {
	int i, dchddfail;
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, kalmanGain, obsCov, 0, uBuf);
	for (i=0; i<MEASUREMENTLENGTH; i++) {
		gsl_matrix_get_col(tempVec, uBuf,i);
		dchddfail = dchdd(STATELENGTH,tempVec, cVec, sVec, cov);
//		cout << "DDCHDD=\n" << dchddfail << "\n";
	}
}


