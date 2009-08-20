package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;


public interface UKFDoubleVectorFunction extends DoubleVectorFunction
{
	/**
	 */
	public void produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector readings, AbstractDoubleMatrix sigmaError, AbstractDoubleMatrix sigmaOut, AbstractDoubleSquareMatrix stateCovar);
}

