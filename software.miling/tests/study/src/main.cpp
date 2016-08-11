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


int substring( const std::string & striA, const std::string & striB );

int substring( const std::string & striA, const std::string & striB, int & posA, int & posB )
{
    int maxLen = 0;
    int maxAtA = -1;
    int maxAtB = -1;
    for ( auto atA=0; atA<striA.size(); atA++ )
    {
        auto atB = 0;
        while ( ( striA[atA] != striB[atB] ) && ( atB < striB.size() ) )
            atB++;
        if ( atB < striB.size() )
        {
            int len = 0;
            int curAtA = atA;
            int curAtB = atB;
            int iterA = atA;
            while ( ( striA[iterA] == striB[atB] ) && ( atB < striB.size() ) && ( iterA < striA.size() ) )
            {
                iterA++;
                atB++;
                len++;
            }
            if ( len > maxLen )
            {
                maxLen = len;
                maxAtA = curAtA;
                maxAtB = curAtB;
            }
        }
    }
    posA = maxAtA;
    posB = maxAtB;
    return maxLen;
}


int main()
{
    std::string a = "abcdef";
    std::string b = "nhbc";
    int posA, posB;
    int len = substring( a, b, posA, posB );
    //std::cout << std::boolalpha << IsDerivedFrom<A, B>() << std::endl;
    //std::cout << std::boolalpha << IsDerivedFrom<C, A>() << std::endl;
    return 0;
}



