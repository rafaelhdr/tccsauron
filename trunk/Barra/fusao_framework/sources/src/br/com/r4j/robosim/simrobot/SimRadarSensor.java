package br.com.r4j.robosim.simrobot;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.robosim.NoiseControl;
import br.com.r4j.robosim.NoiseControlInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RealPoseDependent;
import br.com.r4j.robosim.estimator.Sensor;


public class SimRadarSensor implements Sensor, Configurable, RealPoseDependent, RendererListener, NoiseControl
{
	private static Log log = LogFactory.getLog(SimRadarSensor.class.getName());
	private static Log logSens = LogFactory.getLog("radar");

	private Pose2D poseRobot = null;

	private Pose2D poseRadar = null;
	private double lastRads = Math.PI/2;

	private DoubleSquareMatrix covarRadarMeasure = null;

	private double phoErrorMm = 5.0;
	private double thetaErrorRad = 5*Math.PI/180;

	private double dimPhoError = 0;
	private double dimThetaRadarError = 0;
	private double dimThetaRobotError = 0;

	private Random rnd = null;
	
	private Shape shpTriangle = null;

	private boolean bResults = false;


	public SimRadarSensor()
	{
		covarRadarMeasure = new DoubleSquareMatrix(3);
		poseRadar = new Pose2D(0, 0, 0);
		rnd = new Random((new Date()).getTime());
		
		shpTriangle = ShapesUtil.createTriangle(60*Math.PI/180, 16, 60*Math.PI/180, true);
	}


	public String getName()
	{
		return "Sensor Radar";
	}


	public Pose2D getRadarPose()
	{
		return poseRadar;
	}

	public double getLastRads()
	{
		return lastRads;
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/phoErrorMm"))
			phoErrorMm = props.getDoubleProperty(strBaseKey + "/phoErrorMm").doubleValue();
		if (props.containsProperty(strBaseKey + "/thetaErrorGra"))
			thetaErrorRad = props.getDoubleProperty(strBaseKey + "/thetaErrorGra").doubleValue()*Math.PI/180;
		if (props.containsProperty(strBaseKey + "/radar_x"))
			poseRadar.setX(props.getDoubleProperty(strBaseKey + "/radar_x").doubleValue());
		if (props.containsProperty(strBaseKey + "/radar_y"))
			poseRadar.setY(props.getDoubleProperty(strBaseKey + "/radar_y").doubleValue());

		log.debug("strBaseKey: " + strBaseKey);
		log.debug("poseRadar: " + poseRadar);

		covarRadarMeasure.setElement(0, 0, phoErrorMm*phoErrorMm); covarRadarMeasure.setElement(0, 1, 0); covarRadarMeasure.setElement(0, 2, 0);
		covarRadarMeasure.setElement(1, 0, 0); covarRadarMeasure.setElement(1, 1, thetaErrorRad*thetaErrorRad); covarRadarMeasure.setElement(1, 2, 0); 
		covarRadarMeasure.setElement(2, 0, 0); covarRadarMeasure.setElement(2, 1, 0); covarRadarMeasure.setElement(2, 2, thetaErrorRad*thetaErrorRad*25);
	}


	public void setRealPose(Pose2D realPose)
	{
		log.debug("setRealPose: " + realPose);
		this.poseRobot = realPose;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
		dimPhoError = rnd.nextGaussian()*phoErrorMm*noiseCtrl.noiseLevelScale;
		dimThetaRadarError = rnd.nextGaussian()*thetaErrorRad*noiseCtrl.noiseLevelScale;
		dimThetaRobotError = rnd.nextGaussian()*thetaErrorRad*2.5*noiseCtrl.noiseLevelScale;

		double dX = poseRobot.getX() - poseRadar.getX();
		double dY = poseRobot.getY() - poseRadar.getY();
		if (dX*dX < 0.0001 || dY*dY < 0.0001)
			bResults = false;
		else			
			bResults = true;
	}


	public int getDataDimension()
	{
		return 3;
	}


	public void getData(AbstractDoubleVector output)
	{
		log.debug("poseRobot: " + poseRobot);
		double dX = poseRobot.getX() - poseRadar.getX();
		double dY = poseRobot.getY() - poseRadar.getY();

		if (noiseCtrl.randomReadingFrequency > 0 && Math.random()*100 >= noiseCtrl.randomReadingFrequency)
		{
			int bbb = 2*((int)(Math.random() + 0.50001)) - 1;
			double err = bbb*noiseCtrl.randomReadingScale*(0.5 + Math.random());
			if (dX > 0)
				output.setComponent(0, err*Math.sqrt(dX*dX + dY*dY));
			else
				output.setComponent(0, -err*Math.sqrt(dX*dX + dY*dY));

			bbb = 2*((int)(Math.random() + 0.50001)) - 1;
			err = bbb*noiseCtrl.randomReadingScale*(0.5 + Math.random());
			if (dX*dX > 0.0001)
				if (dX > 0)
					lastRads = FunctionsR.aproxRadian(err*Math.atan(dY/dX), lastRads);
				else
					lastRads = FunctionsR.aproxRadian(err*Math.atan(dY/dX) + Math.PI, lastRads);
			else
				if (dY > 0)
					lastRads = FunctionsR.aproxRadian(err*Math.PI/2, lastRads);
				else
					lastRads = FunctionsR.aproxRadian(err*3*Math.PI/2, lastRads);
			output.setComponent(1, lastRads);
			
			bbb = 2*((int)(Math.random() + 0.50001)) - 1;
			err = bbb*noiseCtrl.randomReadingScale*(0.5 + Math.random());
			output.setComponent(2, err*poseRobot.getTheta());

			log.debug("output: " + MatrixUtil.toString(output, 7, 2));
			logSens.debug(MatrixUtil.toString(output, 7, 2));
		}
		else
		{
			lastRads = FunctionsR.aproxRadian(Math.atan(dY/dX), lastRads);

			if (dX > 0)
				output.setComponent(0, Math.sqrt(dX*dX + dY*dY) + dimPhoError);
			else
				output.setComponent(0, -1*(Math.sqrt(dX*dX + dY*dY) + dimPhoError));

			if (dX*dX > 0.0001)
				if (dX > 0)
					lastRads = FunctionsR.aproxRadian(Math.atan(dY/dX), lastRads);
				else
					lastRads = FunctionsR.aproxRadian(Math.atan(dY/dX) + Math.PI, lastRads);
			else
				if (dY > 0)
					lastRads = FunctionsR.aproxRadian(Math.PI/2, lastRads);
				else
					lastRads = FunctionsR.aproxRadian(3*Math.PI/2, lastRads);
			output.setComponent(1, lastRads + dimThetaRadarError);
			
			output.setComponent(2, poseRobot.getTheta() + dimThetaRobotError);

			log.debug("output: " + MatrixUtil.toString(output, 7, 2));
			logSens.debug(MatrixUtil.toString(output, 7, 2));
		}
	}


	public boolean hasNewData()
	{
		return bResults;
	}



	public AbstractDoubleSquareMatrix getDataCovariance()
	{
		logSens.debug("\r\n" + MatrixUtil.toString(covarRadarMeasure, 9, 4));
		return covarRadarMeasure;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		if (poseRobot != null)
		{
			Graphics2D g2d = e.getGraphics();
	
			Shape shpShp = e.translateAndMaintainShapeSize(poseRadar.getX(), poseRadar.getY(), shpTriangle);
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.green.brighter());
			g2d.fill(shpShp);
			g2d.setColor(Color.green.darker());
			g2d.draw(shpShp);
		}
	}


	public void render(RendererEvent e)
	{
	}


	public void erase(RendererEvent e)
	{
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


