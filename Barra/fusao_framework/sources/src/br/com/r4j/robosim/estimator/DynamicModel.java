
package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface DynamicModel extends BaseModel, DataConsumer
{
	/**
	 * sempre precisa retornar resultados.
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar);


	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance);


//	public AbstractDoubleSquareMatrix correctCovarByDimension(AbstractDoubleSquareMatrix covOld);


	public void correctByDimension(AbstractDoubleVector stateMean, AbstractDoubleSquareMatrix stateCovar,
													AbstractDoubleVector stateMeanNew, AbstractDoubleSquareMatrix stateCovarNew);
}
