#include<stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>

template<typename T> struct Tr: public std::char_traits<T>
{
public:
    static int compare(const T* s1, const T* s2, size_t n)
    {
        for ( auto i=0; i<n; i++ )
        {
            T ss1 = ( ( s1[i] >= 'A' ) && (s1[i] <= 'Z') ) ? (s1[i] + 'a' - 'A') : s1[i];
            T ss2 = ( ( s2[i] >= 'A' ) && (s2[i] <= 'Z') ) ? (s2[i] + 'a' - 'A') : s2[i];

            if ( ss1 < ss2 )
                return -1;
            else if ( ss1 > ss2 )
                return 1;
        }
        return 0;
    }
};

using String = std::basic_string<char, Tr<char> >;

int main(){
    std::cout << (String( "a" ) > String( "A" ) ) << std::endl;

   return 0;
}
