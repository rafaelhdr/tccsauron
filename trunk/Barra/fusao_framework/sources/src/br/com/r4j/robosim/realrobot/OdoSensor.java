package br.com.r4j.robosim.realrobot;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.NoiseControl;
import br.com.r4j.robosim.NoiseControlInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.estimator.Sensor;


public class OdoSensor implements Sensor, Configurable
{
	private Pose2D lastReading = null;
	private Pose2D reading = null;

	private double phoErrorFront4mm = 0.05;
	private double phoErrorTheta4mm = 0.01*Math.PI/180;
	private double thetaError4rad = 0.05;


	public String getName()
	{
		return "Odometer Sensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/phoErrorFront4mm"))
			phoErrorFront4mm = props.getDoubleProperty(strBaseKey + "/phoErrorFront4mm").doubleValue();
		if (props.containsProperty(strBaseKey + "/phoErrorTheta4mm"))
			phoErrorTheta4mm = props.getDoubleProperty(strBaseKey + "/phoErrorTheta4mm").doubleValue()*Math.PI/180;
		if (props.containsProperty(strBaseKey + "/thetaError4Gra"))
			thetaError4rad = props.getDoubleProperty(strBaseKey + "/thetaError4Gra").doubleValue()*Math.PI/180;
	}


	public void setReadings(Pose2D reading)
	{
		this.lastReading = this.reading;
		this.reading = reading;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
		// não precisa fazer nada ...
	}


	public int getDataDimension()
	{
		return 3;
	}


	public void getData(AbstractDoubleVector output)
	{
		double dX = reading.getX() - lastReading.getX();
		double dY = reading.getY() - lastReading.getY();
		double d = Math.sqrt(dX*dX + dY*dY);

		double thetaTransl = 0;
		if (dX*dX > 0.025)
		{
			if (dX > 0)
				thetaTransl = Math.atan(dY/dX);
			else
				thetaTransl = Math.atan(dY/dX) + Math.PI;
		}
		else
		{
			if (dY > 0)
				thetaTransl = Math.PI/2;
			else
				thetaTransl = 3*Math.PI/2;
		}
		double dThetaTrans = thetaTransl - lastReading.getTheta();
		double dTheta = (reading.getTheta() - lastReading.getTheta());

		output.setComponent(0, d);
		output.setComponent(1, dTheta);
		output.setComponent(2, dThetaTrans);
	}


	public boolean hasNewData()
	{
		return lastReading != null;
	}



	public AbstractDoubleSquareMatrix getDataCovariance()
	{
		float dX = (float) (reading.getX() - lastReading.getX());
		float dY = (float) (reading.getY() - lastReading.getY());
		double d = Math.sqrt(dX*dX + dY*dY);
		double dTheta = (reading.getTheta() - lastReading.getTheta());

		double phoErrorFront = d*phoErrorFront4mm; phoErrorFront = phoErrorFront*phoErrorFront;
		double phoErrorPhoTheta = d*phoErrorTheta4mm; phoErrorPhoTheta = phoErrorPhoTheta*phoErrorPhoTheta;
		double thetaError = dTheta*thetaError4rad*Math.PI/180; thetaError = thetaError*thetaError;
		thetaError += phoErrorPhoTheta;

		AbstractDoubleSquareMatrix covarDist = new DoubleSquareMatrix(2);
		covarDist.setElement(0, 0, phoErrorFront);
		covarDist.setElement(1, 1, thetaError);

		return covarDist;
	}
}


