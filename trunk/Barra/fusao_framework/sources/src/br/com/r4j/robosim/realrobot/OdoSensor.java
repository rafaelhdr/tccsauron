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


public class OdoSensor implements Sensor, Configurable, NoiseControl
{
	private static Log log = LogFactory.getLog(OdoSensor.class.getName());
	private static Log logSens = LogFactory.getLog("odom");

	private Pose2D lastReading = null;
	private Pose2D reading = null;

	private double phoErrorFront4mm = 0.05;//*0.05;
	private double thetaError4mm = (0.01*Math.PI/180);//*(0.01*Math.PI/180);
	private double thetaError4rad = 0.05;//*0.05;

	private double d = -1;
	private double dTheta = -1;

	private Random rnd = null;


	public OdoSensor()
	{
		reading = new Pose2D(0, 0, 0);

		rnd = new Random((new Date()).getTime());
	}


	public String getName()
	{
		return "Odometer Sensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/phoErrorFront4mm"))
		{
			phoErrorFront4mm = props.getDoubleProperty(strBaseKey + "/phoErrorFront4mm").doubleValue();
			phoErrorFront4mm = phoErrorFront4mm;//*phoErrorFront4mm;
		}
		if (props.containsProperty(strBaseKey + "/thetaError4mm"))
		{
			thetaError4mm = props.getDoubleProperty(strBaseKey + "/thetaError4mm").doubleValue()*Math.PI/180;
			thetaError4mm = thetaError4mm;//*thetaError4mm;
		}
		if (props.containsProperty(strBaseKey + "/thetaError4Gra"))
		{
			thetaError4rad = props.getDoubleProperty(strBaseKey + "/thetaError4Gra").doubleValue();
			thetaError4rad = thetaError4rad;//*thetaError4rad;
		}
	}


	public void setReadings(Pose2D reading)
	{
		log.debug("setReadings:reading = " + reading);
		this.lastReading = this.reading;
		this.reading = reading;
		log.debug("this.reading = " + this.reading + ", this.lastReading = " + this.lastReading);
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
		this.d += rnd.nextGaussian()*d*phoErrorFront4mm*(noiseCtrl.noiseLevelScale - 1);
		this.dTheta = reading.getTheta() - lastReading.getTheta();
		this.dTheta += rnd.nextGaussian()*Math.abs(dTheta)*thetaError4rad*(noiseCtrl.noiseLevelScale - 1) + 
					   rnd.nextGaussian()*d*thetaError4mm*(noiseCtrl.noiseLevelScale - 1);
// 		this.dTheta = FunctionsR.centerRadian(Math.PI*2 - this.dTheta, 0);
 		this.dTheta = FunctionsR.centerRadian(this.dTheta, 0);

		if (noiseCtrl.randomReadingFrequency > 0)
		{
			double rnd = Math.random();
			if (noiseCtrl.randomReadingFrequency > 0 && rnd*100 >= noiseCtrl.randomReadingFrequency)
			{
				this.d =     noiseCtrl.randomReadingScale*(0.5 + Math.random())*d;
				this.dTheta = noiseCtrl.randomReadingScale*(0.5 + Math.random())*dTheta + 
								  noiseCtrl.randomReadingScale*(0.5 + Math.random())*d*thetaError4rad;
			}
		}
		log.debug("dataAvailable:d, dTheta: " + d + ", " + dTheta);
	}


	public int getDataDimension(BaseModel baseModel)
	{
		return 2;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		logSens.debug("getData:reading = " + reading + ", lastReading = " + lastReading);
		log.debug("getData:reading = " + reading + ", lastReading = " + lastReading);
		output.setComponent(0, d);
		output.setComponent(1, dTheta);
		logSens.debug(MatrixUtil.toString(output, 9, 2));
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		logSens.debug("hasNewData:lastReading != null");
		log.debug("hasNewData:lastReading != null");
		return lastReading != null;
	}


	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		double sigmaDError = d*phoErrorFront4mm;
			   sigmaDError = sigmaDError*sigmaDError;

		double sigmaThetaError = (d*thetaError4mm*d*thetaError4mm +
							      dTheta*thetaError4rad*dTheta*thetaError4rad);

		AbstractDoubleSquareMatrix covarDist = new DoubleSquareMatrix(2);
		covarDist.setElement(0, 0, sigmaDError);
		covarDist.setElement(1, 1, sigmaThetaError);

		return covarDist;
	}


	public void reset()
	{
		reading = new Pose2D(0, 0, 0);
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

