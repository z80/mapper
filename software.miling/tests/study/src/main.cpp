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
#include <hash_map>
//using namespace boost::filesystem;

class Baloons
{
public:
    Baloons( const std::list<int> & vals )
    {
        baloons = vals;
    }

    int theBestOrder( std::list<int> & order )
    {
        //std::cout << "entered: "; 
        //for ( auto i=baloons.begin(); i!=baloons.end(); i++ )
        //    std::cout << *i << ", ";
        //std::cout << std::endl;

        int depth = order.size();
        int bestInd = -1;
        int bestVal = -1;
        int sz = baloons.size();
        if ( sz < 2 )
        {
            int val = baloons.back();
            order.push_back( val );
            return val;
        }
        for ( auto i=0; i!=sz; i++ )
        {
            int j=0;
            auto iter = baloons.begin();
            while ( j++ < i )
                iter++;
            int origVal = *iter;
            int val  = origVal;

            auto prev = iter;
            if ( iter != baloons.begin() )
            {
                prev--;
                val *= *prev;
            }

            auto next = iter;
            if ( iter != baloons.end() )
            {
                next++;
                if ( next != baloons.end() )
                    val *= *next;
            }

            baloons.erase( iter );

            std::list<int> bestList;
            val += theBestOrder( bestList );
            if ( val > bestVal )
            {
                bestInd = i;
                bestVal = val;
                while ( order.size() > depth )
                    order.pop_back();
                order.push_back( origVal );
                while ( bestList.size() )
                {
                    order.push_back( bestList.front() );
                    bestList.pop_front();
                }
            }
            // Return value in it's place.
            
            iter = baloons.begin();
            j=0;
            while ( j++ < i )
                iter++;
            baloons.insert( iter, origVal );
        }

        //for ( auto i=baloons.begin(); i!=baloons.end(); i++ )
        //    std::cout << *i << ", ";
        //std::cout << "val: " << bestVal << std::endl;

        return bestVal;
    }

    std::list<int> baloons;
};




int main()
{
    std::list<int> l;
    l.push_back( 1 );
    l.push_back( 5 );
    l.push_back( 8 );
    l.push_back( 11 );
    l.push_back( 1 );
    l.push_back( 2 );
    l.push_back( 12 );
    Baloons b( l );
    l.clear();
    int val = b.theBestOrder( l );
    for ( auto i=l.begin(); i!=l.end(); i++ )
    {
        std::cout << *i << ", ";
    }
    std::cout << "result: " << val << std::endl;
    return 0;
}






