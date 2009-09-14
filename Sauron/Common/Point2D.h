#ifndef __POINT_2D_H__
#define __POINT_2D_H__

namespace sauron
{
    
template <typename T> class Point2D
{
    public:
        Point2D<T>();
        Point2D<T>( const Point2D<T> &other );
        Point2D<T>( const T &x, const T &y );
        
        //*******************
        //  Getters
        inline T &X();
        inline const T &X() const;
        
        inline T &Y();
        inline const T &Y() const;
        //*******************
        
        //*******************
        //  Operators
        inline Point2D<T> &operator = ( const Point2D<T> &other );
        
        inline Point2D<T> operator + ( const Point2D<T> &other ) const;
        inline Point2D<T> &operator += ( const Point2D<T> &other );
        
        inline Point2D<T> operator - ( const Point2D<T> &other ) const;
        inline Point2D<T> &operator -= ( const Point2D<T> &other );
        
        inline Point2D<T> operator * ( const Point2D<T> &other ) const;
        inline Point2D<T> operator * ( const T &value ) const;
        inline Point2D<T> &operator *= ( const T &value );
        
        inline Point2D<T> operator / ( const Point2D<T> &other ) const;
        inline Point2D<T> operator / ( const T &value ) const;
        inline Point2D<T> &operator /= ( const T &value );
        //*******************

        double getDistance( const Point2D<T> &other ) const;

    
    private:
        T   m_x;
        T   m_y;
};


typedef Point2D<int>    Point2DInt;
typedef Point2D<float>  Point2DFloat;
typedef Point2D<double> Point2DDouble;

    
}; // namespace sauron


#include "Point2DImpl.h"


#endif /* __POINT_2D_H__ */

