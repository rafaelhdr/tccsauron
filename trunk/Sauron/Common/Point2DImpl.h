#ifndef __POINT_2D_IMPLEMENTATION_H__
#define __POINT_2D_IMPLEMENTATION_H__

namespace sauron
{
    
template <typename T> Point2D<T>::Point2D()
    : m_x( (T)0 ),
      m_y( (T)0 )
{
}


template <typename T> Point2D<T>::Point2D( const Point2D<T> &other )
    : m_x( other.m_x ),
      m_y( other.m_y )
{
}


template <typename T>Point2D<T>::Point2D( const T &x, const T &y )
    : m_x( x ),
      m_y( y )
{
}


template <typename T> T &Point2D<T>::X()
{
    return m_x;
}


template <typename T> const T &Point2D<T>::X() const
{
    return m_x;
}


template <typename T> T &Point2D<T>::Y() 
{
    return m_y;
}


template <typename T> const T &Point2D<T>::Y() const
{
    return m_y;
}


template <typename T> Point2D<T> &Point2D<T>::operator = ( const Point2D<T> &other )
{
    m_x = other.m_x;
    m_y = other.m_y;
    
    return (*this);
}


template <typename T> Point2D<T> Point2D<T>::operator + ( const Point2D<T> &other ) const
{
    return Point2D<T>( m_x + other.m_x, m_y + other.m_y );
}


template <typename T> Point2D<T> &Point2D<T>::operator += ( const Point2D<T> &other )
{
    m_x += other.m_x;
    m_y += other.m_y;
    
    return (*this);
}


template <typename T> Point2D<T> Point2D<T>::operator - ( const Point2D<T> &other ) const
{
    return Point2D<T>( m_x - other.m_x, m_y - other.m_y );
}


template <typename T> Point2D<T> &Point2D<T>::operator -= ( const Point2D<T> &other )
{
    m_x -= other.m_x;
    m_y -= other.m_y;
    
    return (*this);
}


template <typename T> Point2D<T> Point2D<T>::operator * ( const T &value ) const 
{
    return Point2D<T>( m_x * value, m_y * value );
}


template <typename T> Point2D<T> &Point2D<T>::operator *= ( const T &value )
{
    m_x *= value;
    m_y *= value;
    
    return (*this);
}


template <typename T> Point2D<T> Point2D<T>::operator / ( const T &value ) const 
{
    return Point2D<T>( m_x / value, m_y / value );
}


template <typename T> Point2D<T> &Point2D<T>::operator /= ( const T &value )
{
    m_x /= value;
    m_y /= value;
    
    return (*this);
}


template <typename T> double Point2D<T>::getDistance( const Point2D<T> &other ) const
{
    double diff_x2 = m_x - other.m_x;
    diff_x2 *= diff_x2;

    double diff_y2 = m_y - other.m_y; 
    diff_y2 *= diff_y2;

	return sqrt(diff_x2 + diff_y2);
}
    
}; // namespace sauron


#endif /* __POINT_2D_IMPLEMENTATION_H__ */

