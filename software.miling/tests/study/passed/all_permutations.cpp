#include<stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>


void printingAllPermutations( const std::string & stri )
{
    std::string s = stri;
    std::sort( s.begin(), s.end() );
    do {
        std::cout << s << std::endl;
    } while ( std::next_permutation( s.begin(), s.end() ) );
}


int main(){
    printingAllPermutations( "Hello!" );
    return 0;
}
