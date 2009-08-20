package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface DynamicModel extends DataConsumer
{
	public String getName();


	public void setSensor(Sensor sns);

	
	public int getDataDimension();

	/**
	 * sempre precisa retornar resultados.
	 */
	public void produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar, AbstractDoubleVector statePredictedOut);


	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance);
}
