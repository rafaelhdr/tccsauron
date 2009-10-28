#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class Kalman
{
    public:
        virtual void getPrioriEstimate( const Matrix &F, const Matrix &Q, 
                                        Pose &estimate,  Matrix &P ) = 0;

        virtual void getPosterioriEstimate( const Matrix &z, const Matrix &H, const Matrix &R, 
                                            Pose &estimate, Matrix &P ) = 0;
};

}   // namespace sauron

#endif // __KALMAN_H__