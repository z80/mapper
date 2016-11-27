/*
************************************************
Header file for UnscentedKalmanFilter.h
David Sachs
Compiles on Microsoft Visual C++ 6.0
************************************************
*/


#include <iostream>
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_cblas.h"
#include "gsl/gsl_blas.h"
#include "gsl/gsl_linalg.h"
#include "stdlib.h"
#include "windows.h"
#include "time.h"
#include <cmath>

#define STATELENGTH 9
#define MEASUREMENTLENGTH 9
#define MOMENTARM 0.3


class UKF {
public:

//Constructor
UKF();
//Destructor
~UKF();

/*
*********************************************************
User Accessible Filter components
*********************************************************
*/


//Filter initialization in which most of the memory used is allocated.
//This should be called once before the filtering begins.
void initFilter();

//Times the quaternion based square-root unscented Kalman filter
double timeFilter(int numTrials);


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
void callFilter(double measurementData1, double measurementData2, double measurementData3,double measurementData4, double measurementData5, double measurementData6, double measurementData7, double measurementData8, double measurementData9, double timeCalled, double magx,double magy, double magz);

//Sets an element of the orientation quaternion
void setQuatStateElement(int elementNumber, double value);

void printNoise();

//Retrieves an element of the state vector
double getStateElement(int elementNumber);

//Retrieves an element of the orientation quaternion
double getQuatStateElement(int elementNumber);

void setMeasurementNoise(int elementNumber, double value);

void setProcessNoise(int elementNumber, double value);


/*
***************************************************
Unscented Kalman Filters
***************************************************
*/


//UKF
void ukf();

//square root UKF
void srukf();

//quaternion based square root UKF
void qsrukf();


private:

//Initialize SRUKF parameters
double lambda, gamma, covWeightZero, covWeightOne, sqrtCovWeightZero, sqrtCovWeightOne;
double alpha;// = 1;//0.01;
double beta;// = 2.0;
double kappa;// = 0;
int numberOfSigmaPoints;
double newTime;// = 0;


//Declare buffers to be used in calculations (minimizing memory allocation
//at runtime

//Declare matrix buffers
gsl_matrix *stateBuf;
gsl_matrix *sigBuf;
gsl_matrix *obsSigBuf;
gsl_matrix *quatBuf;
gsl_matrix *quatErrBuf;
gsl_matrix *wObsSigBuf;
gsl_matrix *updStateBuf;
gsl_matrix *wUpdStateBuf;
gsl_matrix *mWeights;
gsl_matrix *qWWeights;
gsl_matrix *obsMWeights;
gsl_matrix *QRBuf;
gsl_matrix *obsQRBuf;
gsl_matrix *processNoise;
gsl_matrix *measurementNoise;
gsl_matrix *vErrBuf;
gsl_matrix *cov;
gsl_matrix *tempCov;
gsl_matrix *uBuf;
gsl_matrix *inverted;
gsl_matrix *invertedTrans;
gsl_matrix *makeUpperTriangle;
gsl_matrix *obsMakeUpperTriangle;
gsl_matrix *rBuf;
gsl_matrix *obsRBuf;
gsl_matrix *obsCov;
gsl_matrix *obsCovTemp;
gsl_matrix *dchudTestMat;
gsl_matrix *PXY;
gsl_matrix *PXYTemp;
gsl_matrix *kalmanGain;
gsl_matrix *sigBufLong;
gsl_matrix *obsSigBufLong;
gsl_matrix *sigBufShort;
gsl_matrix *obsSigBufShort;
gsl_matrix *sigBufLongTemp;
gsl_matrix *obsSigBufLongTemp;
gsl_matrix *sigBufShortTemp;
gsl_matrix *obsSigBufShortTemp;
gsl_matrix *stateMeans;
gsl_matrix *obsMeans;
gsl_matrix *meanTest;

//Declare Vector Buffers
gsl_vector *stateVector;
gsl_vector *measurementVector;
gsl_vector *state;
gsl_vector *qState;
gsl_vector *stateMean;
gsl_vector *sqCWMean;
gsl_vector *obsSqCWMean;
gsl_vector *obsPredMean;
gsl_vector *tempVec;
gsl_vector *rowVector;
gsl_vector *obsTempVec;
gsl_vector *sVec;
gsl_vector *cVec;
gsl_vector *obsSVec;
gsl_vector *obsCVec;
gsl_vector *dchudVec;
gsl_vector *dchddVec;
gsl_vector *zeroVec;
gsl_vector *measVec;
gsl_vector *rotError;

//Declare quaternion buffers
gsl_vector *posQuatInv;
gsl_vector *errQuat;
gsl_vector *magQuat;
gsl_vector *angVQuat;
gsl_vector *tempQuat;
gsl_vector *gravQuat;	
gsl_vector *tempGravQuat;
gsl_vector *tempMagQuat;
gsl_vector *gravNewQuat;
gsl_vector *magNewQuat;
gsl_vector *tempAngMomQuat;
gsl_vector *angMomNewQuat;
gsl_vector *centAccel;
gsl_vector *tempPosQuat;
gsl_vector *posQuat;
gsl_vector *momentArm;

//Declare permutation
gsl_permutation *perm;


/*
*******************************************************
Vector and matrix display utilities
*******************************************************
*/

//Display a matrix on screen (for debugging only)
void matPrint(gsl_matrix *m); 
//Display a vector on screen (for debugging only)
void vecPrint(gsl_vector *v); 


/*
************************************************************
Basic Mathematical functions
************************************************************
*/

//Perform quaternion multiplication on qLeft and qRight, leaving the result in qLeft
void quatMult(gsl_vector *quatLeft, gsl_vector *quatRight);

//Invert qInitial, and leave the result in qInverse
//(only works if the quaternion is normalized)
void quatInverse(gsl_vector *quatInverse, gsl_vector *qInitial);
//Normalize a quaternion
void quatNormalize(gsl_vector *quat);

//Perform a cholesky factor downdate
//Ported from the LINPACK FORTRAN function DCHDD
int dchdd(int p, gsl_vector *x, gsl_vector *c, gsl_vector *s, gsl_matrix *r); 

//Perform a cholesky factor update
//Ported from the LINPACK FORTRAN function DCHUD
void dchud(int p, gsl_vector *x, gsl_vector *c, gsl_vector *s, gsl_matrix *r); 



/*
*******************************************************
Models used in filter
*******************************************************
*/


//Update the state vector in a quaterion square root UKF
void stateFunction(); 
//Update the observation vector for a quaternion based square root UKF
void observationFunction();



void updateQuat();
void generateSigmaPoints();
void findMeanState();
void findMeanStateQuat();
void updateCovarianceState();
void findMeanObservation();
void findMeanObservationQuat();
void updateCovarianceObservation();
void findKalmanGain();
void finalMean();
void finalCovariance();

};