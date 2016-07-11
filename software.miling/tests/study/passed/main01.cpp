#include<stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

class Base
{
public:
    virtual void method()
    {
        std::cout << "From base" << std::endl;
    }

    void call()
    {
        method();
    }

    Base() {}

    virtual ~Base()
    {
        method();
    }
};

class Derived: public Base
{
public:
    virtual void method()
    {
        std::cout << "From derived" << std::endl;
    }

    Derived()
    {
        method();
    }
    ~Derived()
    {
        method();
    }

};

int main(){
    /*
   char a=250;
   int expr;
   expr= a+ !a + ~a + ++a;
   printf("%d",expr);
   return 0;
   */
   Derived * d = new Derived();
   Base * b = dynamic_cast<Base *>( d );
   d->method();
   b->method();
   d->call();
   b->call();

   delete d;
   return 0;
}
