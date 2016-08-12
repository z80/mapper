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
#include <regex>
#include <sstream>
#include <locale>
#include <iostream>
#include <queue>
//using namespace boost::filesystem;


class SnakesAndLadders
{
public:
    SnakesAndLadders( int width, int height );
    ~SnakesAndLadders();

    void add( int from, int to );
    void shortestPath();
private:
    std::vector<int> visited;
    std::list< std::pair<int, int> > toVisit;
    std::map<int, int> tunnels;
    bool minPath( int from, std::list<int> & path );
    void visitFromHere( int node, std::list<int> & nodes );
};



int main()
{
    SnakesAndLadders sl( 7, 5 );
    sl.add( 1, 30 );
    sl.add( 4, 0 );
    sl.add( 33, 0 );
    sl.add( 32, 0 );
    sl.shortestPath();
    return 0;
}

SnakesAndLadders::SnakesAndLadders( int width, int height )
{
    visited.resize( width* height, false );
}

SnakesAndLadders::~SnakesAndLadders()
{

}

void SnakesAndLadders::shortestPath()
{
    std::list<int> p;
    bool res = minPath( 0, p );
    std::cout << p.size() << std::endl;
    std::for_each( p.begin(), p.end(), [&]( int node )
    {
        std::cout << node << " ";
    });
    std::cout << std::endl;
}

void SnakesAndLadders::add( int from, int to )
{
    tunnels[ from ] = to;
}

bool SnakesAndLadders::minPath( int from, std::list<int> & path )
{
    // In out of range or already visited return failure.
    if ( from >= visited.size() )
        return false;

    // Mark as visited.
    toVisit.push_back( std::pair<int, int>( from, from ) );

    while ( true )
    {
        // Loop around all in queue.
        bool found = false;
        std::for_each( toVisit.begin(), toVisit.end(), [&]( std::pair<int, int> node )
        {
            visited[node.second] = node.first;
            if ( node.second == visited.size()-1 )
                found = true;
        });
        if ( found )
            break;
        // When stuck try to find alternatives.
        std::list< std::pair<int, int> > newQueue;
        std::for_each( toVisit.begin(), toVisit.end(), [&]( std::pair<int, int> node )
        {
            std::list<int> nodes;
            visitFromHere( node.second, nodes );
            std::for_each( nodes.begin(), nodes.end(), [&]( int dest )
            {
                newQueue.push_back( std::pair<int, int>( node.second, dest ) );
            });
        } );
        toVisit = newQueue;
    }

    // Trace back.
    path.clear();
    int node = visited.size()-1;
    path.push_front( node );
    while ( node != from )
    {
        node = visited[ node ];
        path.push_front( node );
    }
    return true;
}

void SnakesAndLadders::visitFromHere( int node, std::list<int> & nodes )
{
    nodes.clear();
    if ( tunnels.find( node ) != tunnels.end() )
        nodes.push_back( tunnels[node] );
    else
    {
        int end = node + 6;
        end = ( end < visited.size() ) ? end : visited.size();
        for ( auto i=node+1; i<end; i++ )
        {
            nodes.push_back( i );
        }
    }
}





