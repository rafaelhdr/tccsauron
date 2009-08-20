package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface Sensor extends DataConsumer
{
	public String getName();

	public int getDataDimension();

	public boolean hasNewData();

	public void getData(AbstractDoubleVector output);

	public AbstractDoubleSquareMatrix getDataCovariance();
}
