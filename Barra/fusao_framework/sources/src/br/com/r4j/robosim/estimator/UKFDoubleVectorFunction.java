package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {61A9B2D0-A92B-4415-B9AE-E9C887750330} */
public interface UKFDoubleVectorFunction extends DoubleVectorFunction
{
	/**
	 * @modelguid {7217F799-2787-4F62-AA0C-F80565A107BC}
	 */
	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector statePred, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar);

	public boolean canProduceObservations(AbstractDoubleVector readings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar);
}

