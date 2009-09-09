#include "VerticalProjectionDetector.h"

#include <vector>
#include <queue>
#include <cmath>
#include "Point2D.h"
#include "DiscretizedLine.h"

namespace sauron
{



VerticalProjectionDetector::VerticalProjectionDetector()
{
}

VerticalProjectionDetector::~VerticalProjectionDetector()
{
}


void VerticalProjectionDetector::detect( const Image &colorImage, const Image &convImage, ProjectionVector &projs ) const
{
    const uint imageWidth  = convImage.getWidth();
    const uint imageHeight = convImage.getHeight();

    const byte startThreshold  = 75;
    const byte stopThreshold   = 25;
    const uint maxLineGap      = 2;
    const uint minLineLenght   = 25;

    const float minLineTan = 11.0f;
    
    const int verticalInertiaInitial = 5;
    int verticalInertia = verticalInertiaInitial;

    projs.clear();

    Image localMaxImage( convImage );
    //Image auxiliarImage( imageWidth, imageHeight, 8, Pixel::PF_GRAY );

    std::vector< DiscretizedLine > linesVec;
    int size = 1;
    for ( register uint j = size; j < imageHeight - size; ++j )
    {
        for ( register uint i = size; i < imageWidth - size; ++i )
        {
            byte maxValue  = 0;
            uint maxValueX = 0;
            uint maxValueY = 0;

            for ( register int n = -size; n <= size; ++n )
            {
                for ( register int m = -size; m <= size; ++m )
                {
                    Pixel pixel = localMaxImage( i + n, j + m );
                    if ( pixel.Gray() > maxValue )
                    {
                        maxValue  = pixel.Gray();
                        maxValueX = i + n;
                        maxValueY = j + m;
                    }
                    pixel.Gray() = 0;
                }
            }
        
            localMaxImage( maxValueX, maxValueY ).Gray() = maxValue;
        }
    }


    for ( register uint j = 1; j < imageHeight - 1; ++j )
    {
        for ( register uint i = 1; i < imageWidth - 1; ++i )
        {
            /*if ( auxiliarImage( i, j ).Gray() > 0 )
                continue;*/

            if ( localMaxImage( i, j ).Gray() < startThreshold )
                continue;

           uint belowThreshold = 0;
           DiscretizedLine possibleNewLine;

           possibleNewLine.addPoint( Point2DInt( i, j ) );
           uint x = i;
           uint y = j;

           std::queue< Point2DInt > belowThrehsoldPoints;

           verticalInertia = verticalInertiaInitial;

           bool keepIterating = true;
           while ( keepIterating ) 
           {
               localMaxImage(x, y).Gray() = 0;

               if ( ++y >= imageHeight )
                   break;

               byte nextPixelValue  = 0;
               uint nextPixelXCoord = 0;

               if ( x > 1 )
               {
                   localMaxImage(x - 1, y).Gray() = 0;
                   if ( convImage(x - 1, y).Gray() > nextPixelValue )
                   {
                       nextPixelValue  = convImage(x - 1, y).Gray();
                       nextPixelXCoord = x - 1;
                   }
               }

               if ( x + 1 < imageWidth )
               {
                   localMaxImage(x + 1, y).Gray() = 0;
                   if ( convImage(x + 1, y).Gray() > nextPixelValue )
                   {
                       nextPixelValue  = convImage(x + 1, y).Gray();
                       nextPixelXCoord = x + 1;
                   }
               }

               localMaxImage(x, y).Gray() = 0;
               if ( convImage(x, y).Gray() > nextPixelValue )
               {
                   nextPixelValue  = convImage(x, y).Gray();
                   nextPixelXCoord = x;
               }
               else if ( --verticalInertia > 0 )
               {
                   if ( belowThrehsoldPoints.size() <= maxLineGap || nextPixelValue < stopThreshold )
                   {
                       nextPixelValue  = convImage(x, y).Gray();
                       nextPixelXCoord = x;
                   }
               }
               else
                   verticalInertia = verticalInertiaInitial;



               if ( nextPixelValue > stopThreshold )
               {
                   while ( belowThrehsoldPoints.size() > 0 )
                   {
                       possibleNewLine.addPoint( belowThrehsoldPoints.front() );
                       belowThrehsoldPoints.pop();
                   }

                   possibleNewLine.addPoint( Point2DInt( nextPixelXCoord, y ) );
               }
               else if ( belowThrehsoldPoints.size() <= maxLineGap )
               {
                   belowThrehsoldPoints.push( Point2DInt( nextPixelXCoord, y ) );
               }
               else
               {
                   keepIterating = false;
               }

               x = nextPixelXCoord;

               if ( possibleNewLine.getNumPoints() >= 10 )
                   if ( abs( possibleNewLine.getTangent() ) < minLineTan )
                       break;
           }

           if ( possibleNewLine.getNumPoints() > minLineLenght && abs( possibleNewLine.getTangent() ) > minLineTan )
           {
               Projection newProj( colorImage, possibleNewLine );
               projs.push_back( newProj );
           }
        }
    }

    //std::vector< DiscretizedLine > modifiedLines;
    //for ( register uint i = 0; i < linesVec.size(); ++i )
    //    modifiedLines.push_back( recalculateLine( linesVec[i] ) );

    //Image out( imageWidth, imageHeight, 32, Pixel::PF_RGB );
    //std::vector< DiscretizedLine >::iterator it;
    //int colorNum = 0;
    //for ( it = linesVec.begin(); it != linesVec.end(); ++it )
    ////for ( it = modifiedLines.begin(); it != modifiedLines.end(); ++it )
    //{           
    //    /*std::cout << "DiscretizedLine " << colorNum << std::endl;
    //    std::cout << "\tAngle: " << (*it).getAngle() / 3.1415259 * 180.0 << std::endl;*/
    //    for ( register uint i = 0; i < (*it).getNumPoints(); ++i )
    //    {   
    //        Point2DInt point = (*it).getPoint( i );
    //        //std::cout << "\tPoint(" << point.X() << "," << point.Y() << ")" << std::endl;
    //        if ( colorNum % 4 == 0)
    //            out( point.X(), point.Y() ).set( 255, 0, 0);
    //        else if ( colorNum % 4 == 1 )
    //            out( point.X(), point.Y() ).set( 0, 255, 0);
    //        else if ( colorNum % 4 == 2 )
    //            out( point.X(), point.Y() ).set( 255, 255, 0);
    //        else
    //            out( point.X(), point.Y() ).set( 0, 255, 255 );
    //    }
    //    ++colorNum;
    //}

    //im = out;
}


DiscretizedLine VerticalProjectionDetector::recalculateLine( DiscretizedLine &line )
{
    Point2DInt start = line.getPoint(0);
    Point2DInt end   = line.getPoint( line.getNumPoints() - 1 ) ;

    DiscretizedLine ret;

    line.sortVertical();
    float deltaX = (float)start.X() - (float)end.X();
    float deltaY = (float)start.Y() - (float)end.Y();

    uint maxY;
    uint minY;

    if ( start.Y() > end.Y() )
    {
        maxY = start.Y();
        minY = end.Y();
    }
    else
    {
        maxY = end.Y();
        minY = start.Y();
    }

    if ( deltaX == 0.0 )
    {
        uint maxY = 0;
        uint minY = 0;
        
        for ( register uint j = minY; j < maxY; ++j )
            ret.addPoint( Point2DInt( start.X(), j ) );
    }
    else
    {
        float m = deltaY / deltaX;

        for ( register uint j = minY; j < maxY; ++j )
            ret.addPoint( Point2DInt( (int)floor((start.X() + (j - start.Y()) / m)), j ) );
    }

    return ret;
}



} // namespace sauron