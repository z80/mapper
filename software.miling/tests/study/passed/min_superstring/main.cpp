#include <stdio.h>
#include <conio.h>
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


int commonSuffix( const std::string & striA, const std::string & striB )
{
    int lenA = striA.size();
    int lenB = striB.size();
    int sz = ( lenA < lenB ) ? lenA : lenB;
    int suffixSz = 0;
    for ( auto i=0; i<sz; i++ )
    {
        auto indA = lenA - i - 1;
        auto indB = i;
        if ( striA[indA] == striB[indB] )
            suffixSz++;
        else
            break;
    }
    return suffixSz;
}

void merge2Strings( const std::string & striA, const std::string & striB, std::string & sum )
{
    int suffixSz = commonSuffix( striA, striB );
    sum.resize( striA.size() + striB.size() - suffixSz );
    for ( auto i=0; i<striA.size(); i++ )
        sum[i] = striA[i];
    for ( auto i=suffixSz; i<striB.size(); i++ )
        sum[ striA.size() + i - suffixSz ] = striB[i];
}

class Strings
{
public:
    Strings();
    ~Strings();

    Strings & operator<<( const std::string & stri );

    void append( const std::string & stri );
    void result( std::string & stri );

private:
    void mergeAll( std::string & sum );
    std::list<std::string> striList;
};

int main()
{
    Strings stris;
    stris << "aaa";
    stris << "abb";
    stris << "bbaaaa";
    stris << "sa";
    stris << "assa";
    stris << "aabbsa";
    stris << "aassas";
    stris << "ass";
    stris << "aaba";
    stris << "aasas";
    stris << "ass";
    std::string sum;
    stris.result( sum );
    std::cout << sum;
    getch();
    return 0;
}


Strings::Strings()
{
}

Strings::~Strings()
{
}

Strings & Strings::operator<<( const std::string & stri )
{
    append( stri );
    return *this;
}

void Strings::append( const std::string & stri )
{
    striList.push_back( stri );
}

void Strings::result( std::string & stri )
{
    striList.sort();
    std::string sum;
    mergeAll( sum );
    std::string bestSum = sum;
    while ( std::next_permutation( striList.begin(), striList.end() ) )
    {
        mergeAll( sum );
        if ( sum.size() < bestSum.size() )
            bestSum = sum;
    }
    stri = bestSum;
}

void Strings::mergeAll( std::string & sum )
{
    auto ind = striList.begin();
    auto indPrev = ind++;
    merge2Strings( *indPrev, *ind, sum );
    while ( ind != striList.end() )
    {
        std::string res;
        merge2Strings( sum, *ind, res );
        sum = res;
        ind++;
    }
}


