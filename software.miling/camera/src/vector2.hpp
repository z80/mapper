
#ifndef __VECTOR2_HPP_
#define __VECTOR2_HPP_

namespace Math
{

template <int DIM = 3, typename FLOAT = double> class Vector
{
public:
    Vector();
    Vector( const Vector<DIM, FLOAT> &);
    Vector( const FLOAT *);
    ~Vector();
    const Vector<DIM, FLOAT> & operator=( const Vector<DIM, FLOAT> & );
    const Vector<DIM, FLOAT> & operator=( const FLOAT * );
    FLOAT & operator[](int index);
    const FLOAT & operator[](int index) const;
private:
    FLOAT val[DIM];
};

template<int DIM, typename FLOAT> Vector<DIM, FLOAT>::Vector()
{
    for ( int i=0; i<DIM; i++ )
        val[i] = static_cast<FLOAT>( 0.0 );
}

template<int DIM, typename FLOAT> Vector<DIM, FLOAT>::Vector( const Vector<DIM, FLOAT> & inst )
{
    for ( int i=0; i<DIM; i++ )
        val[i] = inst.val[i];
}

template<int DIM, typename FLOAT> Vector<DIM, FLOAT>::Vector( const FLOAT * x)
{
    for ( int i=0; i<DIM; i++ )
        val[i] = x[i];
}

template<int DIM, typename FLOAT> Vector<DIM, FLOAT>::~Vector()
{
}

template<int DIM, typename FLOAT> const Vector<DIM, FLOAT> & Vector<DIM, FLOAT>::operator=( const Vector<DIM, FLOAT> & inst )
{
    if ( this != &inst )
    {
        for ( int i=0; i<DIM; i++ )
            val[i] = inst.val[i];
    }
    return *this;
}

template<int DIM, typename FLOAT> const Vector<DIM, FLOAT> & Vector<DIM, FLOAT>::operator=( const FLOAT * v )
{
    for ( int i=0; i<DIM; i++ )
        val[i] = v[i];
    return *this;
}

template<int DIM, typename FLOAT> FLOAT & Vector<DIM, FLOAT>::operator[]( int index )
{
    return val[index];
}

template<int DIM, typename FLOAT> const FLOAT & Vector<DIM, FLOAT>::operator[](int index) const
{
    return val[index];
}

}

#endif


