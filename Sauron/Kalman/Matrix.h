#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <cv.h>
#include <cxcore.h>
#include "BaseTypes.h"

namespace sauron
{

class Matrix
{
    public:
        Matrix( uint r, uint c );
        Matrix( const Matrix &other );
        ~Matrix();

        inline double get( uint r, uint c ) const;
        inline void set( uint r, uint c, double value );

        inline uint getNumRows() const;
        inline uint getNumCols() const;

        inline double &operator () ( uint r, uint c );
        inline Matrix &operator = ( const Matrix &other );
        inline Matrix &operator = ( const CvMat *other );

        inline operator CvMat*();
        inline operator const CvMat*() const;

    private:
        CvMat *m_matrix;
};


}   // namespace sauron

#include "MatrixImpl.h"

#endif // _MATRIX_H_