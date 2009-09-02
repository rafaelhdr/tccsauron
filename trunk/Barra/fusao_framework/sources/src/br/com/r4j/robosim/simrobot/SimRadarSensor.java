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
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;


/** @modelguid {DFF7A1AA-994C-4CF5-A461-7E90F9102A8D} */
public class SimRadarSensor implements Sensor, Configurable, RealPoseDependent, RendererListener, NoiseControl
{
	/** @modelguid {59CCEA68-A5E3-486B-9CAD-93E8B9CBDEC5} */
	private static Log log = LogFactory.getLog(SimRadarSensor.class.getName());
	/** @modelguid {4528E785-9C04-4DB7-B787-4AABEBB73507} */
	private static Log logSens = LogFactory.getLog("radar");

	/** @modelguid {47D02B25-1AD6-458A-8CB0-4A243787CCA2} */
	private Pose2D poseRobot = null;

	/** @modelguid {BD728090-1603-41B0-B787-8E00A98AFB54} */
	private Pose2D poseRadar = null;
	/** @modelguid {A66B7E21-2A36-4592-BDA3-D85CDBD308B7} */
	private double lastRads = Math.PI/2;

	/** @modelguid {EECEE31D-49E8-4485-8057-20C6B6F022E1} */
	private DoubleSquareMatrix covarRadarMeasure = null;

	/** @modelguid {91B68288-BC61-4600-B4CE-17DCDE4C5EB6} */
	private double phoErrorMm = 5.0;
	/** @modelguid {B17BC75D-A046-4E49-ABF0-22EFDCFD896E} */
	private double thetaErrorRad = 5*Math.PI/180;

	/** @modelguid {4B7DABB8-0D61-4F45-ABCA-01317BF92C81} */
	private double dimPhoError = 0;
	/** @modelguid {ABC52154-71FE-4765-BEC9-BB8CE20A2830} */
	private double dimThetaRadarError = 0;
	/** @modelguid {1D11F2B7-2DAE-424F-9043-E47AC339571D} */
	private double dimThetaRobotError = 0;

	/** @modelguid {97EC4A69-4691-43B1-A96B-F442A6D6941C} */
	private Random rnd = null;
	
	/** @modelguid {86503025-8D28-41E3-8493-7B11B4C91ECD} */
	private Shape shpTriangle = null;

	/** @modelguid {6F79F82D-D7F1-4974-8BEE-D99589258B3A} */
	private boolean bResults = false;


	/** @modelguid {0FFB9374-D569-482F-879A-52B1CE061EB5} */
	public SimRadarSensor()
	{
		covarRadarMeasure = new DoubleSquareMatrix(3);
		poseRadar = new Pose2D(0, 0, 0);
		rnd = new Random((new Date()).getTime());
		
		shpTriangle = ShapesUtil.createTriangle(60*Math.PI/180, 16, 60*Math.PI/180, true);
	}


	/** @modelguid {0B9500FE-A5D6-4A69-B2EC-A774A14AE731} */
	public String getName()
	{
		return "Sensor Radar";
	}


	/** @modelguid {153BAA97-71B6-4DC4-AFD4-17411595F97C} */
	public Pose2D getRadarPose()
	{
		return poseRadar;
	}

	/** @modelguid {D179419D-2E21-4993-BAB8-1839C0298006} */
	public double getLastRads()
	{
		return lastRads;
	}


	/** @modelguid {433AF2C7-8A5F-4A46-8E98-77F79FA87D5E} */
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


	/** @modelguid {11A30E9A-77C8-450E-8EB4-9815A3A48931} */
	public void setRealPose(Pose2D realPose)
	{
		log.debug("setRealPose: " + realPose);
		this.poseRobot = realPose;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 * @modelguid {D3E6E256-FB27-4041-99AA-FDA139460F2F}
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


	/** @modelguid {14738F87-4498-4BB4-A604-0126E0B0D8C4} */
	public int getDataDimension(BaseModel baseModel)
	{
		return 3;
	}


	/** @modelguid {CB68A5DF-F400-4B8B-8A05-E96BF256AA9D} */
	public void getData(AbstractDoubleVector output, BaseModel baseModel)
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


	/** @modelguid {21C5623E-3443-473E-975F-31D9B550328B} */
	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		return bResults;
	}



	/** @modelguid {A594B326-3675-4990-823A-BC90C6EECED9} */
	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		logSens.debug("\r\n" + MatrixUtil.toString(covarRadarMeasure, 9, 4));
		return covarRadarMeasure;
	}


	/** @modelguid {29A8ECB4-DC4E-4C5C-A732-61783364F59F} */
	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	/** @modelguid {67F1E35B-7C57-4D6B-94FA-D3685944B429} */
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


	/** @modelguid {CEA450B5-69BF-48A7-92FF-AB461AC833E1} */
	public void render(RendererEvent e)
	{
	}


	/** @modelguid {47083C21-3F15-4A2A-9814-C35BC0A9D228} */
	public void erase(RendererEvent e)
	{
	}


	/** @modelguid {AB3FB3D0-073E-4A73-91CD-32FE8705D60F} */
	private NoiseControlInfo noiseCtrl = new NoiseControlInfo();
	/** @modelguid {FDB9DE20-2425-408F-8960-C5B31827471C} */
	public boolean canControlNoise() {return true;}
	/** @modelguid {39C7BBFA-7E05-4606-AB25-94301CDE31A4} */
	public boolean canAddRandomReadings() {return true;}

	/** @modelguid {F30779A0-A395-4A8E-BB44-DBB2F44AB2D6} */
	public void setNoiseLevel(double noiseScale) {noiseCtrl.noiseLevelScale = noiseScale;}
	/** @modelguid {56F7697E-1AB7-42D6-94BB-CA3A50D1BCE0} */
	public void setRandomLevel(double noiseScale, double noiseFrequency)
	{
		noiseCtrl.randomReadingScale = noiseScale;
		noiseCtrl.randomReadingFrequency = noiseFrequency;
	}

	/** @modelguid {EE106A05-D2F2-4A6A-B02A-0E681D3213C4} */
	public void setNoiseControlInfo(NoiseControlInfo noiseInfo) {this.noiseCtrl = noiseInfo;}
	/** @modelguid {6CEA1EE1-3DBD-4BB1-AA35-86EADA26127F} */
	public NoiseControlInfo getNoiseControlInfo() {return noiseCtrl;}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.Sensor#reset()
	 */
	public void reset()
	{
	}
}


