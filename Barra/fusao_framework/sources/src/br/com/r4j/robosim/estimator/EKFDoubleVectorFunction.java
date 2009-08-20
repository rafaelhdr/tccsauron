package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;


public interface EKFDoubleVectorFunction extends DoubleVectorFunction
{
	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state);
}
