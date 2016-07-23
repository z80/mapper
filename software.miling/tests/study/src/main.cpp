#include<stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>

class C
{
public:
    C()
    {
        protectedInstance = 0;
        en = true;
    }
    C( const C & inst )
    {
        *this = inst;
    }
    virtual ~C()
    {
    }
    const C & operator=( const C & inst )
    {
    }
    static void setPretected( bool en )
    {
        this->en = en;
        if ( !en )
        {
            if ( procectedInstance )
            {
                delete pretectedInstance;
                protectedInstance = 0;
            }
        }
    }



public:
    void * operator new( size_t qty )
    {
        void * ptr = malloc( qty );
        if ( !ptr )
            throw "Err mem allocation";
        if ( en )
            protectedInstance = ptr;
    }
    void  operator delete( void * obj )
    {
        free( obj );
        if ( en )
            protectedInstance = 0;
    }
private:
    static C * protectedInstance;
    static bool en;
};



int main(){
    C::setProtected( true );
    C * c = new C();
    //std::shared_ptr<C> sc( c );
    //c = 0;
    // Even more it is necessary to use mutexes 
    // to prevent user of the code to try do something 
    // with the pointer in a separate thread.
    try {
        // Working code here.
    } except( ... )
    {
    }

    C::setProtected( false );
    return 0;
}



