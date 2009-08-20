package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface SensorModel extends DataConsumer
{
	public String getName();


	public void setSensor(Sensor sns);


	public int getDataDimension();


	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 */
	public void produceResults(AbstractDoubleVector state, AbstractDoubleVector obsPredicted, AbstractDoubleSquareMatrix stateCovar);


	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance);


	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings);


	/**
	 * @param vectReadings
	 * @return
	 */
	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance);
}
