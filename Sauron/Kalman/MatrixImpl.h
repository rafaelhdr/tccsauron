#ifndef _MATRIX_IMPLEMENTATION_H_
#define _MATRIX_IMPLEMENTATION_H_

namespace sauron
{

Matrix::Matrix( uint r, uint c )
{
    m_matrix = cvCreateMat( r, c, CV_64FC1 );
    cvZero( m_matrix );
}


Matrix::Matrix( const Matrix &other )
{
    m_matrix = cvCloneMat( (const CvMat *)other );
}


Matrix::~Matrix()
{
    cvReleaseMat( &m_matrix );
}


uint Matrix::getNumRows() const
{
    return (uint)m_matrix->rows;
}


uint Matrix::getNumCols() const
{
    return (uint)m_matrix->cols;
}


double Matrix::get( uint r, uint c ) const
{
    return cvmGet( m_matrix, r, c );
}


void Matrix::set( uint r, uint c, double value )
{
    cvmSet( m_matrix, r, c, value );
}


double &Matrix::operator ()( uint r, uint c )
{
    int   step   = m_matrix->step / sizeof(double);
    double *data = m_matrix->data.db;

    return (data + c * step)[r];
}


Matrix &Matrix::operator = ( const Matrix &other )
{
    cvReleaseMat( &m_matrix );
    m_matrix = cvCloneMat( other );
}


Matrix &Matrix::operator = ( const CvMat *other )
{
    cvReleaseMat( &m_matrix );
    m_matrix = cvCloneMat( other );
}


Matrix::operator CvMat *()
{
    return m_matrix;
}


Matrix::operator const CvMat *() const
{
    return m_matrix;
}


}

#endif  _MATRIX_IMPLEMENTATION_H_