package br.com.r4j.robosim;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;


public abstract class BaseSonarSensor implements Sensor, Configurable
{
        // matriz 8x8 onde todos os elementos valem readingSigma ^ 2
	private AbstractDoubleSquareMatrix covar = null;
        // alcance do sonar (pode ser configurado)
	private double maxRange = 3000;
        // valor padrão de readingSigma, pode ser configurado
	private double readingSigma = 50;
        // ângulo de abertura do cone do sonar, em radianos (pode ser configurado)
	private double beta = 30*Math.PI/180;
        // número de sonares
	protected int sonarCount = 8;

        // o ângulo dos sonares, em relação à base do robô
	protected double [] arrayThetaS = null;
        // o cosseno dos ângulos dos sonares
	protected double [] arrayCosThetaS = null;
        // o seno dos ângulos dos sonares
	protected double [] arraySinThetaS = null;
        // as distâncias de cada sonar à base do robô
	protected double [] arrayDS = null;
        // a coordenada x de cada sonar em relação à base do robô
	protected double [] arrayXS = null;
        // a coordenada y de cada sonar em relação à base do robô
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
	 * M�todo invocado quando os dados estiverem dispon�veis.
	 */
	public void dataAvailable()
	{
		// n�o precisa fazer nada ...
	}


	public int getDataDimension(BaseModel baseModel)
	{
		return 8;
	}


	public abstract void getData(AbstractDoubleVector output, BaseModel baseModel);

	public abstract boolean hasNewData(BaseModel baseModel, Estimator est);

	
	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		return covar;
	}


	public boolean isOutOfRange(double reading)
	{
		return reading >= getMaxReading() || reading <= 0;
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

