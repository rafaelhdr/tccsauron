package br.com.r4j.robosim.simrobot;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.BaseSonarSensor;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.MapDependent;
import br.com.r4j.robosim.NoiseControl;
import br.com.r4j.robosim.NoiseControlInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RealPoseDependent;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.RobotPlayerListener;
import br.com.r4j.robosim.WorldMap;


public class SimSonarSensor extends BaseSonarSensor implements RobotPlayerListener, MapDependent, RealPoseDependent, EstimatorRenderer, RendererListener, NoiseControl
{
	private static Log log = LogFactory.getLog(SimRadarSensor.class.getName());
	private static Log logSens = LogFactory.getLog("sonar");

	private Pose2D poseRobot = null;
	private WorldMap map = null;

	private int idxCurrentSonar = 0;
	private AbstractDoubleVector readings = null;
	private AbstractDoubleVector readingsRenderer = null;
	private boolean bDataProcessed = false;
	
	private Random rnd = null;
	private double phoError = 0;
	

	public SimSonarSensor()
	{
		super();
			
		readings = new DoubleVector(sonarCount);
		readingsRenderer = new DoubleVector(sonarCount);

		rnd = new Random((new Date()).getTime());
	}


	public String getName()
	{
		return "Sonar Sensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		super.configure(props, strBaseKey);
	}


	public void actionStarted(RobotPlayerEvent e)
	{
//		idxCurrentSonar = (idxCurrentSonar + 1)%(sonarCount + 3);
		idxCurrentSonar = (idxCurrentSonar + 1)%(sonarCount + 0);
		logSens.debug("actionStarted: " + idxCurrentSonar);
		if (this.hasNewData())
		{
			for (int i = 0; i < sonarCount; i++)
				readings.setComponent(i, -1);
		}
		bDataProcessed = false;
	}


	public void dataAvailable()
	{
		logSens.debug("dataAvailable");
		super.dataAvailable();
		phoError = rnd.nextGaussian()*this.getReadingSigma()*noiseCtrl.noiseLevelScale;
	}
	

	public void getData(AbstractDoubleVector output)
	{
		logSens.debug("getData:this.hasNewData(): " + this.hasNewData());
		if (this.hasNewData())
		{
			if (!bDataProcessed)
			{
				double currentReading = this.findClosestWallReading(poseRobot.getX(), poseRobot.getY(), poseRobot.getTheta()) + phoError;
				logSens.debug("getData(" + idxCurrentSonar + "): " + currentReading + ", for pose: " + poseRobot);
				
				if (noiseCtrl.randomReadingFrequency > 0 && Math.random()*100 >= noiseCtrl.randomReadingFrequency)
				{
					logSens.debug("noise?: if (noiseCtrl.randomReadingFrequency > 0 && Math.random()*100 >= noiseCtrl.randomReadingFrequency)");
					int bbb = 2*((int)(Math.random() + 0.50001)) - 1;
					double err = bbb*noiseCtrl.randomReadingScale*(0.5 + Math.random());

					readings.setComponent(idxCurrentSonar, err*currentReading);
					readingsRenderer.setComponent(idxCurrentSonar, err*currentReading);
				}
				else
				{
					readings.setComponent(idxCurrentSonar, currentReading);
					readingsRenderer.setComponent(idxCurrentSonar, currentReading);
				}
				this.newPose(poseRobot);
				bDataProcessed = true;
			}
			
			for (int i = 0; i < sonarCount; i++)
				output.setComponent(i, readings.getComponent(i));
		}
	}

	public boolean hasNewData()
	{
		return idxCurrentSonar < sonarCount;
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


	public void setRealPose(Pose2D realPose)
	{
		logSens.debug("setRealPose: " + realPose);
		this.poseRobot = realPose;
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
	}


	private int currentStep = -1;
	private ArrayList listPoses = new ArrayList();
	private ArrayList listReadings = new ArrayList();
	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
	}


	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(20);} catch (Exception e) {}
		}
		listPoses.add(pose);
		listReadings.add(MatrixUtil.clone(readingsRenderer));
		currentStep = listPoses.size() - 1;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (listPoses.size() > 0)
		{
			Pose2D pose = (Pose2D) listPoses.get(currentStep);
			AbstractDoubleVector readingsRendererRndr = (AbstractDoubleVector) listReadings.get(currentStep); 

			Graphics2D g2d = e.getGraphics();
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.green);

			for (int i = 0; i < sonarCount; i++)
			{
				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  
				g2d.drawLine((int) (pose.getX() + arrayDS[i]*Math.cos(arrayThetaS[i] + pose.getTheta())), (int) (pose.getY() + arrayDS[i]*Math.sin(arrayThetaS[i] + pose.getTheta())), (int) x2, (int) y2);
			}	
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		if (listPoses.size() > 0)
		{
			Pose2D pose = (Pose2D) listPoses.get(currentStep);
			AbstractDoubleVector readingsRendererRndr = (AbstractDoubleVector) listReadings.get(currentStep); 

			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.green);

			for (int i = 0; i < sonarCount; i++)
			{
				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  
				g2d.drawLine((int) (pose.getX() + arrayDS[i]*Math.cos(arrayThetaS[i] + pose.getTheta())), (int) (pose.getY() + arrayDS[i]*Math.sin(arrayThetaS[i] + pose.getTheta())), (int) x2, (int) y2);
			}	
		}
		bRendering = false;
	}


	public void erase(RendererEvent e)
	{
		bRendering = true;
		if (listPoses.size() > 0)
		{
			Pose2D pose = (Pose2D) listPoses.get(currentStep);
			AbstractDoubleVector readingsRendererRndr = (AbstractDoubleVector) listReadings.get(currentStep); 

			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.green);

			for (int i = 0; i < sonarCount; i++)
			{
				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  
				g2d.drawLine((int) (pose.getX() + arrayDS[i]*Math.cos(arrayThetaS[i] + pose.getTheta())), (int) (pose.getY() + arrayDS[i]*Math.sin(arrayThetaS[i] + pose.getTheta())), (int) x2, (int) y2);
			}	
		}
		bRendering = false;
	}


	public void actionCompleted(RobotPlayerEvent e)
	{
	}


	public void endOfActions(RobotPlayerEvent e)
	{
	}


	public void beginOfActions(RobotPlayerEvent e)
	{
	}


	public void actionsUpdated(RobotPlayerEvent e)
	{
	}


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


	/**
	 *
	 * 1 - Prucura todas a paredes com inclinação dentro da zona de busca de inclinações.
	 * 2 - Seleciona as paredes que possuem distância a x dentro da faixa de erro.
	 * 3 - seleciona a parede com centor de massa mais próximo do sonar.
	 *
	 * @param double xR
	 * @param double yR
	 * @param double thetaR
	 * @param double reading
	 * @return
	 */
	private double findClosestWallReading(double xR, double yR, double thetaR)
	{
		double cosThetaR = Math.cos(thetaR);
		double sinThetaR = Math.sin(thetaR);

		double beta = this.getBeta();
		double thetaMin = thetaR + arrayThetaS[idxCurrentSonar] - beta/2;
		double thetaMax = thetaMin + beta;

		double xSA = xR + arrayXS[idxCurrentSonar]*cosThetaR - arrayYS[idxCurrentSonar]*sinThetaR;
		double ySA = yR + arrayYS[idxCurrentSonar]*cosThetaR + arrayXS[idxCurrentSonar]*sinThetaR;

		double xOtherSide = 1.5*this.getMaxReading()*Math.cos(thetaR + arrayThetaS[idxCurrentSonar]) + xSA;
		double yOtherSide = 1.5*this.getMaxReading()*Math.sin(thetaR + arrayThetaS[idxCurrentSonar]) + ySA;

		logSens.debug("findClosestWallReading:" + xSA + ":" + ySA + ":" + xOtherSide + ":" + yOtherSide);
		logSens.debug("findClosestWallReading::thetaMin-thetaMax:" + thetaMin + ":" + thetaMax);

		double reading = map.findReadingToClosestCrossingWall(thetaMin, thetaMax, xSA, ySA, xOtherSide, yOtherSide);
		
		if (reading < this.getMaxReading())
			return reading;
		else
			return 1.5*this.getMaxReading();
	}
}


