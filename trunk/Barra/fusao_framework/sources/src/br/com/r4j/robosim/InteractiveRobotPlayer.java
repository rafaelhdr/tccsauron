package br.com.r4j.robosim;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import br.com.r4j.robosim.estimator.Sensor;


/** @modelguid {353CB346-6EA4-40FF-A3A2-2DE03AEF3DD0} */
public class InteractiveRobotPlayer extends RobotPlayer
{
	/** @modelguid {B3A42A5A-C3E8-40C9-865F-8437C795628A} */
	private static Log log = LogFactory.getLog(InteractiveRobotPlayer.class.getName());


	/** @modelguid {5ADEEF06-4051-49BF-927D-1109A8330C03} */
	protected void gatherData(boolean bEngine)
	{
	}


	/** @modelguid {42398F7C-1830-4584-BA42-059CA86C1B0D} */
	public int getNumberOfSteps()
	{
		return 0;
	}


	/** @modelguid {11E2C031-97DB-446B-94F0-358CB99B826F} */
	public void sensorAdded(Sensor sns)
	{
	}


	/** @modelguid {FBC7B342-E149-4531-BF02-AC532DE6F4D4} */
	public void resetBuffers()
	{
	}


	protected AbstractDoubleVector getActualRealPose()
	{
		return null;
	}

}
