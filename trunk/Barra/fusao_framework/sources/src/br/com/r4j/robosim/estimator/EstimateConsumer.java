package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface EstimateConsumer
{
	/**
	 * Indica a necessidade de retornar a estimativa final gerada.
	 *
	 */
	public void estimateAvailable(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar);
}
