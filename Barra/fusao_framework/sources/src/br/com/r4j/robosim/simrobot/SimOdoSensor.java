package br.com.r4j.robosim.simrobot;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.*;
import br.com.r4j.robosim.estimator.Sensor;


public class SimOdoSensor implements Sensor, Configurable, RealPoseDependent, MapDependent, NoiseControl
{
	private static Log log = LogFactory.getLog(SimOdoSensor.class.getName());
	private static Log logSens = LogFactory.getLog("odom");

	private Pose2D lastReading = null;
	private Pose2D reading = null;

	private double dError = 0;
	private double thetaError = 0;
	private double thetaTransError = 0;

	private double d = -1;
	private double dThetaTrans = -1;
	private double dTheta = -1;

	private double phoErrorFront4mm = 0.05;
	private double phoErrorTheta4mm = 0.01*Math.PI/180;
	private double thetaError4rad = 0.05;

	private WorldMap map = null;
	private Random rnd = null;


	public SimOdoSensor()
	{
		rnd = new Random((new Date()).getTime());
		reading = new Pose2D(0, 0, 0);
	}


	public String getName()
	{
		return "Odometer Sensor";
	}


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
		AbstractDoubleVector loc = map.getInitialLocalization();
		reading = new Pose2D(loc.getComponent(0), loc.getComponent(1), loc.getComponent(2));
		log.debug("setWorldMap:reading: " + reading);
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/phoErrorFront4mm"))
			phoErrorFront4mm = props.getDoubleProperty(strBaseKey + "/phoErrorFront4mm").doubleValue();
		if (props.containsProperty(strBaseKey + "/phoErrorTheta4mm"))
			phoErrorTheta4mm = props.getDoubleProperty(strBaseKey + "/phoErrorTheta4mm").doubleValue()*Math.PI/180;
		if (props.containsProperty(strBaseKey + "/thetaError4Gra"))
			thetaError4rad = props.getDoubleProperty(strBaseKey + "/thetaError4Gra").doubleValue()	;
	}


	public void setRealPose(Pose2D realPose)
	{
//		log.debug("setRealPose: realPose = " + realPose);
		this.lastReading = this.reading;
		this.reading = realPose;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 */
	public void dataAvailable()
	{
		double dX = reading.getX() - lastReading.getX();
		double dY = reading.getY() - lastReading.getY();
		this.d = Math.sqrt(dX*dX + dY*dY);

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
		this.dThetaTrans = thetaTransl - lastReading.getTheta();
		this.dTheta = reading.getTheta() - lastReading.getTheta(); 
//		this.dTheta = this.dTheta - dThetaTrans;

		this.dError = rnd.nextGaussian()*d*phoErrorFront4mm*noiseCtrl.noiseLevelScale;
		this.thetaTransError = rnd.nextGaussian()*d*phoErrorTheta4mm*noiseCtrl.noiseLevelScale;
		this.thetaError = rnd.nextGaussian()*Math.abs(dTheta)*thetaError4rad*noiseCtrl.noiseLevelScale;
		
		if (noiseCtrl.randomReadingFrequency > 0)
		{
			double rnd = Math.random();
			if (rnd*100 >= noiseCtrl.randomReadingFrequency)
			{
				this.dError = noiseCtrl.randomReadingScale*(0.5 + Math.random())*d;
				this.thetaTransError = noiseCtrl.randomReadingScale*(0.5 + Math.random())*dThetaTrans;
				this.thetaError = noiseCtrl.randomReadingScale*(0.5 + Math.random())*dTheta;
			}
		}

		log.debug("dError: " + dError);
		log.debug("thetaTransError: " + thetaTransError);
		log.debug("thetaError: " + thetaError);
	}


	public int getDataDimension()
	{
		return 3;
	}


	public void getData(AbstractDoubleVector output)
	{
		output.setComponent(0, d + dError);
		output.setComponent(1, dThetaTrans + thetaTransError);
		output.setComponent(2, dTheta + thetaError + thetaTransError);
		logSens.debug(MatrixUtil.toString(output, 9, 2));
	}


	public boolean hasNewData()
	{
		return lastReading != null;
	}


	public AbstractDoubleSquareMatrix getDataCovariance()
	{
		double sigmaDError = d*phoErrorFront4mm*noiseCtrl.noiseLevelScale; sigmaDError = sigmaDError*sigmaDError;
		double sigmaThetaTransError = d*phoErrorTheta4mm*noiseCtrl.noiseLevelScale; sigmaThetaTransError = sigmaThetaTransError*sigmaThetaTransError;
		double sigmaThetaError = dTheta*thetaError4rad*noiseCtrl.noiseLevelScale; sigmaThetaError = sigmaThetaError*sigmaThetaError;

		AbstractDoubleSquareMatrix covarDist = new DoubleSquareMatrix(3);
		covarDist.setElement(0, 0, sigmaDError);
		covarDist.setElement(1, 1, sigmaThetaTransError);
		covarDist.setElement(2, 2, sigmaThetaError + sigmaThetaTransError);

		return covarDist;
	}


	private NoiseControlInfo noiseCtrl = new NoiseControlInfo();
	public boolean canControlNoise() {return true;}
	public boolean canAddRandomReadings() {return true;}

	public void setNoiseLevel(double noiseScale) {noiseCtrl.noiseLevelScale = noiseScale;}
	public void setRandomLevel(double noiseScale, double noiseFrequency)
	{
		noiseCtrl.randomReadingScale = noiseScale;
		noiseCtrl.randomReadingFrequency = noiseFrequency;
	}

	public void setNoiseControlInfo(NoiseControlInfo noiseInfo) {this.noiseCtrl = noiseInfo;}
	public NoiseControlInfo getNoiseControlInfo() {return noiseCtrl;}
}


