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

class A
{
public:
    A() {}
    ~A() {}

    int i;
    int j;
};

void * operator new( size_t sz )
{
    void * p = malloc( sz );
    std::cout << "new" << std::endl;
    return p;
}

void operator delete( void * p )
{
    std::cout << "delete" << std::endl;
    free( p );
}


int main()
{
    A * a = new A();
    delete a;
    return 0;
}



