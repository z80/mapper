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
//#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string_regex.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/regex.hpp>
#include <regex>
#include <sstream>
#include <locale>
#include <iostream>
#include <queue>
#include <map>
//using namespace boost::filesystem;

// Longest path in matrix avoiding obstacles.

int pathFromPoint( const std::vector< std::vector<int> > & A, std::vector< std::vector<bool> > & visited, std::list< std::pair<int,int> > & path, int y, int x )
{
    if ( ( visited[y][x] ) || ( A[y][x] != 0 ) )
        return 0;

    visited[y][x] = true;

    // And check neighbour points.
    int maxLen = 0;
    std::list< std::pair<int,int> > p, maxPath;
    if ( x > 0 )
    {
        int len = pathFromPoint( A, visited, p, y, x-1 );
        maxLen = len;
        maxPath = p;
    }
    if ( x < A[0].size()-1 )
    {
        p.clear();
        int len = pathFromPoint( A, visited, p, y, x+1 );
        if ( maxLen < len )
        {
            maxLen = len;
            maxPath = p;
        }
    }
    if ( y > 0 )
    {
        p.clear();
        int len = pathFromPoint( A, visited, p, y-1, x );
        if ( maxLen < len )
        {
            maxLen = len;
            maxPath = p;
        }
    }
    if ( y < A.size()-1 )
    {
        p.clear();
        int len = pathFromPoint( A, visited, p, y+1, x );
        if ( maxLen < len )
        {
            maxLen = len;
            maxPath = p;
        }
    }

    // Clear visitness of current point.
    visited[y][x] = false;

    // Append path with current point and found path from current point.
    int len = 1 + maxPath.size();
    path.push_back( std::pair<int, int>( y, x ) );
    while ( maxPath.size() > 0 )
    {
        path.push_back( maxPath.front() );
        maxPath.pop_front();
    }

    return len;
}

int maxPath( const std::vector< std::vector<int> > & A, std::list< std::pair<int,int> > & path )
{
    std::vector< std::vector<bool> > visited;
    std::for_each( A.begin(), A.end(), [&]( const std::vector<int> & a )
    {
        std::vector<bool> v;
        v.resize( a.size(), false );
        visited.push_back( v );
    } );

    int rows = A.size();
    int cols = A[0].size();
    int maxLen = 0;
    std::list< std::pair<int,int> > p;
    for ( int y=0; y<rows; y++ )
    {
        for ( int x=0; x<cols; x++ )
        {
            p.clear();
            int len = pathFromPoint( A, visited, p, y, x );
            if ( len > maxLen )
            {
                maxLen = len;
                path = p;
            }
        }
    }

    return maxLen;
}



int main()
{
    std::vector< std::vector<int> > A;
    std::vector<int> a;
    a.push_back( 0 ); a.push_back( 0 ); a.push_back( 1 ); a.push_back( 1 );
    A.push_back( a );
    a.clear();
    a.push_back( 0 ); a.push_back( 1 ); a.push_back( 0 ); a.push_back( 0 );
    A.push_back( a );
    a.clear();
    a.push_back( 0 ); a.push_back( 0 ); a.push_back( 0 ); a.push_back( 1 );
    A.push_back( a );
    a.clear();
    a.push_back( 1 ); a.push_back( 0 ); a.push_back( 0 ); a.push_back( 1 );
    A.push_back( a );
    a.clear();

    std::list< std::pair<int,int> > path;
    int sz = maxPath( A, path );
    std::cout << "path size: " << sz << std::endl;
    std::for_each( path.begin(), path.end(), [&]( std::pair<int, int> & arg ) 
    {
        std::cout << "( " << arg.first << ", " << arg.second << " ), ";
    } );
    std::cout << std::endl;

    return 0;
}






