#include <stdio.h>
//#include <conio.h>
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
#include <boost/regex.hpp>
#include <regex>
#include <sstream>
#include <locale>
#include <iostream>
#include <queue>
#include <map>
//using namespace boost::filesystem;


// Wildcart match.
bool match( const std::string & stri, const std::string & wildcart, int striInd=0, int wildcartInd=0 )
{
    if ( striInd >= stri.size() )
        return true;
    if ( wildcartInd >= wildcart.size() )
        return true;

    bool res = false;
    if ( wildcart[wildcartInd] == '?' )
        res = match( stri, wildcart, striInd+1, wildcartInd+1 );
    else if ( wildcart[wildcartInd] == '*' )
    {
        res = match( stri, wildcart, striInd+1, wildcartInd+1 ) ||
              match( stri, wildcart, striInd+1, wildcartInd );
    }
    else
        res = ( wildcart[wildcartInd] == stri[striInd] ) &&
              match( stri, wildcart, striInd+1, wildcartInd+1 );
    return res;
}


int main()
{
    std::string stri = "abba";
    std::string wildcart = "??ba";
    std::cout << std::boolalpha << match( stri, wildcart ) << std::endl;
    std::cout << std::boolalpha << match( "qwerty", "qwer*" ) << std::endl;
    return 0;
}






