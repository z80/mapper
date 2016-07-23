#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>

class C
{
public:
    int i;
    int j;

    C()
    {
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
    static void setProtected( bool en )
    {
        C::en = en;
        if ( !en )
        {
            if ( protectedInstance )
            {
                delete protectedInstance;
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
            protectedInstance = reinterpret_cast<C *>( ptr );
        return ptr;
    }
    void  operator delete( void * obj )
    {
        if ( !en )
        {
            free( obj );
            protectedInstance = 0;
        }
    }
private:
    static C * protectedInstance;
    static bool en;
};

C * C::protectedInstance = 0;
bool C::en = true;



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
    } catch( ... )
    {
    }

    delete c;
    c->i = 12345;
    c->j = c->i;

    C::setProtected( false );
    return 0;
}



