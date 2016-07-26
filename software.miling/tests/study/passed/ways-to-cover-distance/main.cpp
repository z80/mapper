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
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <boost/filesystem.hpp>
#include <regex>
using namespace boost::filesystem;

// Getting number of ways to cover distance in three nonzero steps.

int qty( int L )
{
    if ( L == 1 )
        return 1;
    if ( L <= 0 )
        return 0;

    int n = 0;
    for ( auto i=1; i<(L+1); i++ )
    {
        n += qty( L - i );
    }
    return n+1;
}

// Getting number of ways to conver distance with steps of lengths 1, 2, 3.
int qtyL( int L )
{
    if ( L <= 0 )
        return 0;
    if ( L == 1 )
        return 1;
    int n = qtyL( L-1 ) + qtyL( L-2 ) + qty( L-4 ) + 1;
    return n;
}


int main()
{
    std::cout << std::setw( 5 ) << qty( 1 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 2 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 3 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 4 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 5 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 6 ) << std::endl;
    std::cout << std::setw( 5 ) << qty( 7 ) << std::endl;
    std::cout << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 1 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 2 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 3 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 4 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 5 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 6 ) << std::endl;
    std::cout << std::setw( 5 ) << qtyL( 7 ) << std::endl;

    return 0;
}



