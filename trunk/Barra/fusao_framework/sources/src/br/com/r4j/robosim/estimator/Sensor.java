package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface Sensor extends DataConsumer
{
	public String getName();

	public void reset();


	public int getDataDimension(BaseModel baseModel);

	public boolean hasNewData(BaseModel snsModel, Estimator est);

	public void getData(AbstractDoubleVector output, BaseModel snsModel);

	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel snsModel);
}
