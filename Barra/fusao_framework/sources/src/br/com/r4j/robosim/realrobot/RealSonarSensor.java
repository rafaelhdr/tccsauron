package br.com.r4j.robosim.realrobot;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Random;
import java.util.Date;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.Arrays;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.BaseSonarSensor;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RealPoseDependent;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.model.SonarModel;
import br.com.r4j.robosim.NoiseControl;
import br.com.r4j.robosim.NoiseControlInfo;


public class RealSonarSensor extends BaseSonarSensor implements EstimatorRenderer, RealPoseDependent, RendererListener
{
	private static Log log = LogFactory.getLog(RealSonarSensor.class.getName());
	private static Log logSens = LogFactory.getLog("sonar");

	private int [] readings = null;
	private boolean [] arrayBNewDataRndr = null;
//	private boolean [] arrayBNewData = null;
//	private boolean bDataProcessed = false;
	private AbstractDoubleVector readingsRenderer = null;
	private Pose2D poseRobot = null;
	private HashMap mapData = new HashMap();

	private Random rnd = null;


	public RealSonarSensor()
	{
		super();
		readingsRenderer = new DoubleVector(sonarCount);
//		arrayBNewData = new boolean[sonarCount];
		arrayBNewDataRndr = new boolean[sonarCount];
		for (int i = 0; i < sonarCount; i++)
		{
//			arrayBNewData[i] = false;
			arrayBNewDataRndr[i] = false;
		}

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


	public void setReadings(int [] readings)
	{
		this.readings = readings;
		logSens.debug("setReadings: " + Arrays.toString(readings));
		for (int i = 0; i < sonarCount; i++)
		{
			if (readings[i] != -1)
			{
				Iterator itData = mapData.keySet().iterator();
				while (itData.hasNext())
				{
					Object key = itData.next();
					Data2Model mdlData = (Data2Model) mapData.get(key);
					mdlData.bDataProcessed = false;
					mdlData.arrayBNewData[i] = true;
				}
				arrayBNewDataRndr[i] = true;
			}
			else
				arrayBNewDataRndr[i] = false;
		}
		for (int i = 0; i < sonarCount; i++)
		{
			if (readings[i] != -1)
			{
				if (readings[i] < this.getMaxReading())
//					readingsRenderer.setComponent(i, readings[i] - arrayDS[i]);
					readingsRenderer.setComponent(i, readings[i]);
				else
					readingsRenderer.setComponent(i, readings[i]*1.5);	
			}
		}
	}


	class Data2Model
	{
		public boolean bDataProcessed = true;
		public boolean [] arrayBNewData = new boolean[sonarCount];
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		SonarModel snsModel = (SonarModel) baseModel;
		int idxSonar = snsModel.getSonarIndex();

		double phoExtraError = rnd.nextGaussian()*this.getReadingSigma()*(noiseCtrl.noiseLevelScale - 1);
		log.debug("phoExtraError: " + phoExtraError);

		Data2Model mdlData = (Data2Model) mapData.get(snsModel);
		if (mdlData == null)
		{
			mdlData = new Data2Model();
			mdlData.bDataProcessed = false;
			mapData.put(snsModel, mdlData);
			for (int i = 0; i < sonarCount; i++)
				if (readings[i] != -1)
					mdlData.arrayBNewData[i] = true;
		}
		
		if (this.hasNewData(baseModel, null))
		{
			mdlData.arrayBNewData[idxSonar] = false;
			if (!mdlData.bDataProcessed)
			{
				mdlData.bDataProcessed = true;
			}
			for (int i = 0; i < sonarCount; i++)
			{
				if (noiseCtrl.randomReadingFrequency > 0 && Math.random()*100 >= noiseCtrl.randomReadingFrequency)
				{
					if (logSens.isDebugEnabled())
						logSens.debug("noise?: if (noiseCtrl.randomReadingFrequency > 0 && Math.random()*100 >= noiseCtrl.randomReadingFrequency)");
					int bbb = 2*((int)(Math.random() + 0.50001)) - 1;
					double err = bbb*noiseCtrl.randomReadingScale*(0.5 + Math.random());
					output.setComponent(i, err*readings[i]);
				}
				else
				{
					if (readings[i] < this.getMaxReading())
						output.setComponent(i, readings[i] + phoExtraError);
					else
						output.setComponent(i, readings[i]*1.5);
				}
			}
			logSens.debug("output: " + MatrixUtil.toString(output, 5, 2));
		}
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		SonarModel snsModel = (SonarModel) baseModel;
		int idxSonar = snsModel.getSonarIndex();

		Data2Model mdlData = (Data2Model) mapData.get(snsModel);
		if (mdlData == null)
		{
			mdlData = new Data2Model();
			mapData.put(snsModel, mdlData);
		}

		return mdlData.arrayBNewData[idxSonar];
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
	}


	private int currentStep = -1;
	private ArrayList listPoses = new ArrayList();
	private ArrayList listReadings = new ArrayList();
	private boolean [] arrayBNewDataOld = new boolean[8];
	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
	}


	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		logSens.debug("newPose: pose: " + pose);
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(20);} catch (Exception e) {}
		}
		listPoses.add(pose);
		listReadings.add(MatrixUtil.clone(readingsRenderer));
		currentStep = listPoses.size() - 1;
	}


	public double getMaxReading()
	{
		return super.getMaxReading() - 1;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (listPoses.size() > 0 && currentStep > 0)
		{
			Pose2D pose = (Pose2D) listPoses.get(currentStep);
			AbstractDoubleVector readingsRendererRndr = (AbstractDoubleVector) listReadings.get(currentStep); 

			Graphics2D g2d = e.getGraphics();
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			BasicStroke strokeObjectOutlineNew = new BasicStroke(5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 1, new float[] {15, 15}, 0);
			for (int i = 0; i < sonarCount; i++)
			{
				arrayBNewDataOld[i] = arrayBNewDataRndr[i];
//				arrayBNewDataRndr[i] = false;

				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  

				if (arrayBNewDataOld[i])
					g2d.setStroke(strokeObjectOutlineNew);
				else
					g2d.setStroke(strokeObjectOutline);
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
			BasicStroke strokeObjectOutlineNew = new BasicStroke(5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 1, new float[] {15, 15}, 0);
			for (int i = 0; i < sonarCount; i++)
			{
				arrayBNewDataOld[i] = arrayBNewDataRndr[i];
//				arrayBNewDataRndr[i] = false;

				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  

				if (arrayBNewDataOld[i])
					g2d.setStroke(strokeObjectOutlineNew);
				else
					g2d.setStroke(strokeObjectOutline);
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
			BasicStroke strokeObjectOutlineNew = new BasicStroke(5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 1, new float[] {15, 15}, 0);
			for (int i = 0; i < sonarCount; i++)
			{
				double reading = readingsRendererRndr.getComponent(i);
				if (reading >= this.getMaxReading())
					g2d.setColor(Color.red);
				else
					g2d.setColor(Color.green);
				double x2 = pose.getX() + (arrayDS[i] + reading)*Math.cos(arrayThetaS[i] + pose.getTheta());  
				double y2 = pose.getY() + (arrayDS[i] + reading)*Math.sin(arrayThetaS[i] + pose.getTheta());  

				if (arrayBNewDataRndr[i])
					g2d.setStroke(strokeObjectOutlineNew);
				else
					g2d.setStroke(strokeObjectOutline);
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


	public void setRealPose(Pose2D realPose)
	{
		this.poseRobot = realPose;
		this.newPose(poseRobot);
	}


	public void reset()
	{
		readingsRenderer = new DoubleVector(sonarCount);
		for (int i = 0; i < sonarCount; i++)
		{
			Iterator itData = mapData.keySet().iterator();
			while (itData.hasNext())
			{
				Object key = itData.next();
				Data2Model mdlData = (Data2Model) mapData.get(key);
				mdlData.bDataProcessed = false;
				mdlData.arrayBNewData[i] = false;
			}
			arrayBNewDataOld[i] = false;
		}
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


