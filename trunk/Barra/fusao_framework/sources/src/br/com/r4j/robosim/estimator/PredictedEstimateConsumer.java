package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface PredictedEstimateConsumer
{
	/**
	 * Indica a necessidade de retornar a estimativa final gerada.
	 *
	 */
	public void predictedEstimateAvailable(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar);
}
