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

class Dijkstra
{
public:
    std::vector<std::list<int>>  edges;
    std::vector<std::list<int>>  weights;
    std::list<int>                S, notS; // Vects still not in S.
    std::vector<int>              minD; // MinDist.
    std::vector<int>              parents;
    int inf;

    void assignInitWeights( int root );
    int iterationStep( int newlyAddedVert );
    void run( int root );

    void addNode()
    {
        edges.resize( edges.size() + 1 );
        weights.resize( edges.size() );
        minD.resize( edges.size() );
        parents.resize(  edges.size() );
    }

    void addEdge( int from, int to, int weight )
    {
        int sz = (from < to) ? to : from;
        if ( edges.size() < sz )
            edges.resize( sz );
        edges[from].push_back( to );
        if ( weights.size() < sz )
            weights.resize( sz );
        weights[from].push_back( weight );
    }
};

void Dijkstra::assignInitWeights( int root )
{
    // Look for sum of all weights.
    int s = 0;
    std::for_each( edges.begin(), edges.end(), [&]( std::list<int> & ees )
    {
        std::for_each( ees.begin(), ees.end(), [&]( int d )
        {
           s += d;
        } );
    } );
    s += 1;
    this->inf = s;
    // Now s is unreachable infinity.
    // Move unprocessed verts to notS.
    int ind = 0;
    std::for_each( minD.begin(), minD.end(), [&]( int & v )
    {
        if ( ind != root )
        {
            v = s;
            notS.push_back( ind );
        }
        else
        {
            v = 0;
            S.push_back( ind );
        }
        ++ind;
    } );

}

int Dijkstra::iterationStep( int newlyAddedVert )
{
    // Check it's all edges and update weights.
    std::list<int> & edges = this->edges[newlyAddedVert];
    std::list<int> & w      = this->weights[newlyAddedVert];
    int ind = 0;
    auto weightIter = w.begin();
    std::for_each( edges.begin(), edges.end(), [&]( int & v )
    {
        int w = *weightIter;
        int newD = minD[newlyAddedVert] + w;
        if ( minD[v] > newD )
        {
            minD[v] = newD;
            parents[v] = newlyAddedVert;
        }
        ++ind;
        ++weightIter;
    } );

    // Among all adjacent verts add the closest one.
    // And take it out of "notS" and assign it distance weight.
    int bestD = this->inf;
    int bestParentV = -1;
    std::list<int>::iterator bestVIter = std::end(notS);
    for( auto i=notS.begin(); i!=notS.end(); i++ )
    {
        int v = *i;
        // Search edges leading to this vert.
        std::for_each( S.begin(), S.end(), [&]( int & ind )
        {
            std::list<int> & edges = this->edges[ind];
            std::list<int> & weights = this->weights[ind];
            std::list<int>::iterator weightIter = weights.begin();
            std::for_each( edges.begin(), edges.end(), [&]( int & e )
            {
                int weight = *weightIter;
                if ( e == v )
                {
                    bestD = weight;
                    bestVIter = i;
                    bestParentV = ind;
                }
                ++weightIter;
            } );
        } );
        // If it is found
        if ( bestVIter != std::end(notS) )
        {
            // Add it to S. And remove from notS.
            S.push_back( *bestVIter );
            notS.erase( bestVIter );
            // Assign it distance and parent.
            minD[ *bestVIter ] = minD[ bestParentV ] + bestD;
            parents[ *bestVIter ] = bestParentV;
            return *bestVIter;
        }
        else
        {
            // No ways left.
            notS.clear();
            return -1;
        }
    }
}

void Dijkstra::run( int root )
{
    assignInitWeights( root );
    int v = root;
    while ( ( notS.size() > 0 ) && ( v >= 0 ) )
        v = iterationStep( v );
}



int main()
{
    Dijkstra d;
    d.addNode();
    d.addNode();
    d.addNode();
    d.addNode();
    d.addEdge( 0, 1, 1 );
    d.addEdge( 0, 2, 3);
    d.addEdge( 2, 3, 3);
    d.run( 0 );
    std::for_each( d.parents.begin(), d.parents.end(), [&]( int & ind )
    {
        std::cout << "parent is: " << std::setw( 2 ) << ind << std::endl;
    } );
    return 0;
}



