package br.com.r4j.robosim;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.Sensor;


public abstract class BaseSonarSensor implements Sensor, Configurable
{
	private AbstractDoubleSquareMatrix covar = null;
	private double maxRange = 3000;
	private double readingSigma = 50;
	private double beta = 30*Math.PI/180;

	protected int sonarCount = 8; 
	protected double [] arrayThetaS = null;
	protected double [] arrayCosThetaS = null;
	protected double [] arraySinThetaS = null;
	protected double [] arrayDS = null;
	protected double [] arrayXS = null;
	protected double [] arrayYS = null;


	public BaseSonarSensor()
	{
		covar = new DoubleSquareMatrix(8);
		for (int i = 0; i < 8; i++)
			covar.setElement(i, i, readingSigma*readingSigma);

		arrayThetaS = new double[sonarCount];
		arrayCosThetaS = new double[sonarCount];
		arraySinThetaS = new double[sonarCount];
		arrayDS = new double[sonarCount];
		arrayXS = new double[sonarCount];
		arrayYS = new double[sonarCount];

		arrayXS[0] = 115; arrayXS[1] = 155; arrayXS[2] = 190; arrayXS[3] = 210; 
		arrayXS[4] = 210; arrayXS[5] = 190; arrayXS[6] = 155; arrayXS[7] = 115;

		arrayYS[0] = 130; arrayYS[1] = 115; arrayYS[2] = 80; arrayYS[3] = 25; 
		arrayYS[4] = -25; arrayYS[5] = -80; arrayYS[6] = -115; arrayYS[7] = -130;

		arrayThetaS[0] = 90.0 * Math.PI / 180; arrayThetaS[1] = 50.0 * Math.PI / 180; 
		arrayThetaS[2] = 30.0 * Math.PI / 180; arrayThetaS[3] = 10.0 * Math.PI / 180; 
		arrayThetaS[4] = -10.0 * Math.PI / 180; arrayThetaS[5] = -30.0 * Math.PI / 180; 
		arrayThetaS[6] = -50.0 * Math.PI / 180; arrayThetaS[7] = -90.0 * Math.PI / 180;

		for (int i = 0; i < sonarCount; i++)
		{
			arrayDS[i] = Math.sqrt(arrayXS[i]*arrayXS[i] + arrayYS[i]*arrayYS[i]);
			arrayCosThetaS[i] = Math.cos(arrayThetaS[i]);
			arraySinThetaS[i] = Math.sin(arrayThetaS[i]);
		}
	}


	public String getName()
	{
		return "Sonar Sensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/sonar_error"))
		{
			readingSigma = props.getDoubleProperty(strBaseKey + "/sonar_error").doubleValue();
			for (int i = 0; i < 8; i++)
				covar.setElement(i, i, readingSigma*readingSigma);
		}
		if (props.containsProperty(strBaseKey + "/max_range"))
			maxRange = props.getDoubleProperty(strBaseKey + "/max_range").doubleValue();
		if (props.containsProperty(strBaseKey + "/beta"))
			beta = props.getDoubleProperty(strBaseKey + "/beta").doubleValue()*Math.PI/180;
	}


	public double getThetaS(int idx)	{return arrayThetaS[idx];}
	public double getCosThetaS(int idx)	{return arraySinThetaS[idx];}
	public double getSinThetaS(int idx)	{return arraySinThetaS[idx];}
	public double getDS(int idx)	{return arrayDS[idx];}
	public double getXS(int idx)	{return arrayXS[idx];}
	public double getYS(int idx)	{return arrayYS[idx];}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 */
	public void dataAvailable()
	{
		// não precisa fazer nada ...
	}


	public int getDataDimension()
	{
		return 8;
	}


	public abstract void getData(AbstractDoubleVector output);

	public abstract boolean hasNewData();

	
	public AbstractDoubleSquareMatrix getDataCovariance()
	{
		return covar;
	}


	public boolean isOutOfRange(double reading)
	{
		return reading >= maxRange;
	}


	/**
	 * @return
	 */
	public double getReadingSigma()
	{
		return readingSigma;
	}


	/**
	 * @return
	 */
	public double getBeta()
	{
		return beta;
	}
	

	public double getMaxReading()
	{
		return maxRange;
	}
}


