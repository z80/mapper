#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <iomanip>
//#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string_regex.hpp>
//#include <boost/filesystem.hpp>
#include <regex>
#include <sstream>
#include <locale>
#include <iostream>
//using namespace boost::filesystem;

const int ABS_MAX = 10;
const int DIFF  = 1;
// Minimum array adjustment.

bool valid( int * A, int n )
{
    bool correct = true;
    for ( auto i=1; i<n; i++ )
    {
        if ( abs(A[i] - A[i-1]) != DIFF )
            return false;
    }
    return true;
}

bool overflow( int * A, int n )
{
    for ( auto i=0; i<n; i++ )
    {
        if ( A[i] > ABS_MAX )
            return false;
    }
}

int spent( int * A, int n )
{
    if ( n <= 1 )
        return 0;
    if ( n == 2 )
    {
        if ( A[0] < A[1] )
        {
            int sum = A[1] - A[0] - 1;
            A[0] = A[0] + sum;
            return sum;
        }
        else
        {
            int sum = A[0] - A[1] - 1;
            A[1] = A[1] + sum;
            return sum;
        }
    }

    int szRest = spent( A+1, n-1 );
    if ( A[0] < A[1] )
    {
        int sum = A[1] - A[0] - 1;
        A[0] = A[0] + sum;
        return sum+szRest;
    }
    else
    {
        int sum = A[0] - A[1] - 1;
        for ( auto i=1; i<n; i++ )
            A[i] = A[i] + sum;
        return sum*(n-1) + szRest;
    }

    return 0;
}


int main()
{
    int A[] = { 3, 2, 6, 0, 1 };
    int sz = spent( A, sizeof(A)/sizeof(int) );
    std::cout << sz << std::endl;
    return 0;
}



