
#ifndef __MATRIX2_HPP_
#define __MATRIX2_HPP_

#include "vector2.hpp"

namespace Math
{

template <int DIM = 3, typename FLOAT = double> class Matrix
{
public:
    Matrix();
    Matrix( const Matrix<DIM, FLOAT> & );
    ~Matrix();
    const Matrix<DIM, FLOAT> & operator=( const Matrix<DIM, FLOAT> & );
    const Matrix<DIM, FLOAT>   operator*( const Matrix<DIM, FLOAT> & ) const;
    const Vector<DIM, FLOAT>   operator*( const Vector<DIM, FLOAT> & ) const;
    FLOAT * operator[]( int index );
    const FLOAT * operator[]( int index ) const;
    FLOAT det() const;
    const Matrix<DIM, FLOAT> inv() const;
    const Matrix<DIM, FLOAT> transpose() const;
private:
    FLOAT val[ DIM * DIM ];
};

template<int DIM, typename FLOAT> Matrix<DIM, FLOAT>::Matrix()
{
    int index = 0;
    for ( int i=0; i<DIM; i++ )
    {
        for ( int j=0; j<DIM; j++ )
        {
            val[index] = ( i != j ) ? static_cast<FLOAT>( 0.0 ) : static_cast<FLOAT>( 1.0 );
            index += 1;
        }
    }
}

template<int DIM, typename FLOAT> Matrix<DIM, FLOAT>::Matrix( const Matrix<DIM, FLOAT> & inst )
{
    int index = 0;
    for ( int i=0; i<DIM; i++ )
    {
        for ( int j=0; j<DIM; j++ )
        {
            val[index] = inst.val[index];
            index += 1;
        }
    }
}

template<int DIM, typename FLOAT> Matrix<DIM, FLOAT>::~Matrix()
{
}

template<int DIM, typename FLOAT> const Matrix<DIM, FLOAT> & Matrix<DIM, FLOAT>::operator=( const Matrix<DIM, FLOAT> & inst )
{
    if ( this != &inst )
    {
        int index = 0;
        for ( int i=0; i<DIM; i++ )
        {
            for ( int j=0; j<DIM; j++ )
            {
                val[index] = inst.val[index];
                index += 1;
            }
        }
    }
    return *this;
}

template<int DIM, typename FLOAT> const Matrix<DIM, FLOAT> Matrix<DIM, FLOAT>::operator*( const Matrix<DIM, FLOAT> & b ) const
{
    const Matrix<DIM, FLOAT> & a = *this;
    Matrix<DIM, FLOAT> res;
    for ( int iy=0; iy<DIM; iy++ )
    {
        for ( int ix=0; ix<DIM; ix++ )
        {
            FLOAT v = static_cast<FLOAT>( 0.0 );
            for ( int i=0; i<DIM; i++ )
            {
                v += a[iy][i] * b[i][ix];
            }
            res[iy][ix] = v;
        }
    }
    return res;
}

template<int DIM, typename FLOAT> const Vector<DIM, FLOAT> Matrix<DIM, FLOAT>::operator*( const Vector<DIM, FLOAT> & b ) const
{
    const Matrix<DIM, FLOAT> & a = *this;
    Vector<DIM, FLOAT> res;
    for ( int iy=0; iy<DIM; iy++ )
    {
        FLOAT v = static_cast<FLOAT>( 0.0 );
        for ( int i=0; i<DIM; i++ )
        {
            v += (a[iy])[i] * b[i];
        }
        res[iy] = v;
    }
    return res;
}

template<int DIM, typename FLOAT> FLOAT * Matrix<DIM, FLOAT>::operator[]( int index )
{
    return &val[ DIM * index ];
}

template<int DIM, typename FLOAT> const FLOAT * Matrix<DIM, FLOAT>::operator[]( int index ) const
{
    return &val[ DIM * index ];
}

template<int DIM, typename FLOAT> FLOAT Matrix<DIM, FLOAT>::det() const
{
    return static_cast<FLOAT>( 1.0 );
}

template<typename FLOAT> static FLOAT ffabs( FLOAT arg )
{
    return ( arg >= static_cast<FLOAT>( 0.0 ) ) ? arg : -arg;
}

template<int DIM, typename FLOAT> const Matrix<DIM, FLOAT> Matrix<DIM, FLOAT>::inv() const
{
    // ����������� ��������.
    Matrix<DIM, FLOAT> a = *this;
    Matrix<DIM, FLOAT> ia;

    // ��������� ��� ������� ����������.
    for ( int n=0; n<DIM; n++ )
    {
        // ���� ������� �������, �� �������� ����� ��������� �����.
        FLOAT v = a[n][n];
        if ( ffabs<FLOAT>( v ) == static_cast<FLOAT>( 0.0 ) )
        {
            // � ������� �������� ���������.
            for ( int iy=0; iy<DIM; iy++ )
            {
                if ( iy == n )
                    continue;
                for ( int ix=0; ix<DIM; ix++ )
                {
                    a[n][ix]  += a[n][ix];
                    ia[n][ix] += ia[iy][ix];
                }
                v = a[n][n];
                if ( ffabs<FLOAT>( v ) != static_cast<FLOAT>( 0.0 ) )
                    break;
            }
        }
        // ����� ������ �� v.
        for ( int ix=0; ix<DIM; ix++ )
        {
            //int ind = ix + DIM * n;
            a[n][ix] /= v;
            ia[n][ix] /= v;
        }
        // �������� �������� ��� ������� ��������� n.
        for ( int iy=n+1; iy<DIM; iy++ )
        {
            //int indDec = n + DIM * iy;
            FLOAT coef = a[iy][n];
            // ��������� ��� ���� ��������� ������ �������.
            for ( int ix=0; ix<DIM; ix++ )
            {
                //int indSub = ix + DIM * n;
                //int indDec = ix + DIM * iy;
                a[iy][ix]  -= coef * a[n][ix];
                ia[iy][ix] -= coef * ia[n][ix];
            }
        }
        // �������� �������� ��� ������� ���������.
        for ( int iy=0; iy<=(n-1); iy++ )
        {
            //int indDec = n + DIM * iy;
            FLOAT coef = a[iy][n];
            // ��������� ��� ���� ��������� ������.
            for ( int ix=0; ix<DIM; ix++ )
            {
                //int indSub = ix + DIM * n;
                //int indDec = ix + DIM * iy;
                a[iy][ix]  -= coef * a[n][ix];
                ia[iy][ix] -= coef * ia[n][ix];
            }
        }
    }
    return ia;
}

template<int DIM, typename FLOAT>const Matrix<DIM, FLOAT> Matrix<DIM, FLOAT>::transpose() const
{
    Matrix<DIM, FLOAT> ta = *this;
    for ( int n=0; n<DIM-1; n++ )
    {
        for ( int m=n+1; m<DIM; m++ )
        {
            FLOAT b = ta[n][m];
            ta[n][m] = ta[m][n];
            ta[m][n] = b;
        }
    }
    return ta;
}

}


#endif



