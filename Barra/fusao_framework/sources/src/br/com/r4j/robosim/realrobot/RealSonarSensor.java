package br.com.r4j.robosim.realrobot;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.BaseSonarSensor;
import br.com.r4j.robosim.estimator.Sensor;


public class RealSonarSensor extends BaseSonarSensor
{
	private int [] readings = null;
	private boolean bNewData = false;


	public RealSonarSensor()
	{
		super();
	}


	public String getName()
	{
		return "Sonar Sensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		super.configure(props, strBaseKey);
	}


	public void setReadings(int [] readings)
	{
		this.readings = readings;
		for (int i = 0; i < 8; i++)
		{
			if (readings[i] != -1)
			{
				bNewData = true;
				break;
			}
		}
	}


	public void getData(AbstractDoubleVector output)
	{
		if (bNewData)
		{
			bNewData = false;
			for (int i = 0; i < 8; i++)
				output.setComponent(i, readings[i]);
		}
	}

	public boolean hasNewData()
	{
		return bNewData;
	}
}


