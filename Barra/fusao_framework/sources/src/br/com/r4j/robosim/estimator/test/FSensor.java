package br.com.r4j.robosim.estimator.test;

import java.io.IOException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;


public class FSensor implements Sensor, Configurable
{
	private static Log log = LogFactory.getLog(FSensor.class.getName());


	public FSensor()
	{
	}


	public String getName()
	{
		return "FSensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
	}


	public void loadData() throws IOException
	{
	}


	public void resetData()
	{
	}


	public void dataAvailable()
	{
	}


	public int getDataDimension(BaseModel baseModel)
	{
		return 2;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		output.setComponent(0, 0);
		output.setComponent(1, 0);
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		return true;
	}


	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		AbstractDoubleSquareMatrix covarDist = new DoubleSquareMatrix(2);
		covarDist.setElement(0, 0, 0.001);
		covarDist.setElement(1, 1, 0);
		return covarDist;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.Sensor#reset()
	 */
	public void reset()
	{
	}
}


