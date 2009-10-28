#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <cv.h>
#include <cxcore.h>

namespace sauron
{

class Matrix
{
    public:
        Matrix( uint r, uint c );
        Matrix( const Matrix &other );
        Matrix( const Pose &pose );
        ~Matrix();

        inline double get( uint r, uint c ) const;
        inline void set( uint r, uint c, double value );

        inline uint getNumRows() const;
        inline uint getNumCols() const;

        inline double &operator () ( uint r, uint c );
        inline Matrix &operator = ( const Matrix &other );

        inline Matrix operator + ( const Matrix &other ) const;
        inline Matrix &operator += ( const Matrix &other );

        inline Matrix operator - ( const Matrix &other ) const;
        inline Matrix &operator -= ( const Matrix &other );

        inline Matrix operator * ( const Matrix &other ) const;
        inline Matrix operator * ( const double &value ) const;
        inline Matrix &operator *= ( const Matrix &other );
        inline Matrix &operator *= ( const double &value );
};


}   // namespace sauron

//#include "MatrixImpl.h"

#endif // _MATRIX_H_