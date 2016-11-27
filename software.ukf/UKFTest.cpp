/*
******************************************************
Test Program for the UKF class
David Sachs
Compiles on Microsoft Visual C++ 6.0


This is not actually used in practice, as the functions
are called directly from Python instead. This is a short 
program that simply times the filter (0.4 milliseconds
on my 2.2GHz laptop).
*******************************************************
*/

#include "UnscentedKalmanFilter.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

void main()
{
	UKF ukf;
	ukf.initFilter();
	double result;
	double aKey;

	cout << "Timing filter\n";
	result = ukf.timeFilter(1000);
	cout << "The average iteration time is " << result << "\n";
	cout << "Press a key to continue\n";
	cin >> aKey;
}