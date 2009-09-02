package br.com.r4j.robosim.realrobot;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import java.util.Random;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.robosim.NoiseControl;
import br.com.r4j.robosim.NoiseControlInfo;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.math.FunctionsR;
import java.util.Date;


public class FixedOdoSensor implements Sensor
{
	private static Log log = LogFactory.getLog(FixedOdoSensor.class.getName());
	private static Log logSens = LogFactory.getLog("odom");

	private Pose2D lastReading = null;
	private Pose2D reading = null;

	private AbstractDoubleSquareMatrix qFix = null;

	private double d = -1;
	private double dTheta = -1;


	public FixedOdoSensor()
	{
		reading = new Pose2D(0, 0, 0);
	}


	public String getName()
	{
		return "Fixed Odometer Sensor";
	}


	public void setCovariance(AbstractDoubleSquareMatrix qFix)
	{
		this.qFix = qFix;
	}


	public void setReadings(Pose2D reading)
	{
		log.debug("setReadings:reading = " + reading);
		this.lastReading = this.reading;
		this.reading = reading;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
		double dX = reading.getX() - lastReading.getX();
		double dY = reading.getY() - lastReading.getY();

		this.d = Math.sqrt(dX*dX + dY*dY);
		this.dTheta = reading.getTheta() - lastReading.getTheta();
// 		this.dTheta = FunctionsR.centerRadian(Math.PI*2 - this.dTheta, 0);
 		this.dTheta = FunctionsR.centerRadian(this.dTheta, 0);
	}


	public int getDataDimension(BaseModel baseModel)
	{
		return 2;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		log.debug("getData:reading = " + reading + ", lastReading = " + lastReading);
		output.setComponent(0, d);
		output.setComponent(1, dTheta);
		log.debug("output = " + output.getComponent(0) + ", " + output.getComponent(1));
		logSens.debug(MatrixUtil.toString(output, 9, 2));
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		logSens.debug("hasNewData:lastReading != null");
		return lastReading != null;
	}


	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		return qFix;
	}


	public void reset()
	{
		reading = new Pose2D(0, 0, 0);
	}
}

