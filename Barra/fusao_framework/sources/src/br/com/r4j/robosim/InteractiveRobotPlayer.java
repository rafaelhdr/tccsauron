package br.com.r4j.robosim;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.robosim.estimator.Sensor;


public class InteractiveRobotPlayer extends RobotPlayer
{
	private static Log log = LogFactory.getLog(InteractiveRobotPlayer.class.getName());


	protected void gatherData()
	{
	}


	public int getNumberOfSteps()
	{
		return 0;
	}


	public void sensorAdded(Sensor sns)
	{
	}


	public void resetBuffers()
	{
	}
}
