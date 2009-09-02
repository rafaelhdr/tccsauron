package br.com.r4j.jmr.test;

import java.io.IOException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.log4j.Appender;
import org.apache.log4j.Level;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import org.apache.log4j.PatternLayout;
import org.apache.log4j.RollingFileAppender;
import org.apache.log4j.spi.LoggerRepository;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.jmr.datasource.RobotDescription;
import br.com.r4j.research.pose2destimation.Pose2DGrabber;
import br.com.r4j.robosim.Pose2D;


/**
 *  em conf:
 *	- odo-acq/robot-connect
 *	- odo-acq/log-file-name
 *	- odo-acq/pioneer-conf
 *
 * em log4j.xml:
 *	- dataaq.log
 *
 *
 */
public class RespostaVaziaPoseGrabber implements Pose2DGrabber, Runnable
{
	private static Log logResp = LogFactory.getLog("RespostaVaziaPoseGrabber");
	private static Log log = LogFactory.getLog("br.com.r4j.jmr.debug");

	private Log logData = null;
	private Log logDataPoses = null;

	private RobotDescription robot = null;
	private Pose2D poseEstimateTotalOld = null;


	public Pose2D getPose(long imageTimestamp)
	{
		logDataPoses.debug("r:(" + 0 + "," + 0 + "," + 0 + "):dt:" + imageTimestamp);

		poseEstimateTotalOld = new Pose2D(0, 0, 0);
		return poseEstimateTotalOld;
	}


	public Pose2D getMovement(long imageTimestamp)
	{
		logDataPoses.debug("r:(" + 0 + "," + 0 + "," + 0 + "):dt:" + imageTimestamp);
		return new Pose2D(0, 0, 0);
	}


	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp)
	{
		return null;
	}


	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp)
	{
		AbstractDoubleSquareMatrix covarInc = new DoubleSquareMatrix(3);
		covarInc.setElement(0, 0, 100*robot.getDXVariancePerMilimiter());
		covarInc.setElement(1, 1, 100*robot.getDYVariancePerMilimiter());
		covarInc.setElement(2, 2, 0.2*robot.getDThetaVariancePerRad());
		return covarInc;
	}


	public void initEngine()
	{
		robot = new RobotDescription();
		try
		{
			PropertiesHolder props = br.com.r4j.configurator.Configurator.getPropsHolder();

			String strLogName = props.getStringProperty("odo-acq-from-vid/log-file-name");
			if (!strLogName.startsWith("/"))
				strLogName = "/" + strLogName;
			String strLogFileName = props.getStringProperty("log4j/log4j-logs-location") + strLogName;
			strLogName = props.getStringProperty("odo-acq-from-vid/pose-log-file-name");
			if (!strLogName.startsWith("/"))
				strLogName = "/" + strLogName;
			String strPoseLogFileName = props.getStringProperty("log4j/log4j-logs-location") + strLogName;

			// Configura logger de dados
			LoggerRepository repository = LogManager.getLoggerRepository();
			Logger cat = repository.getLogger("dataaq.log");
			cat.setAdditivity(false);
			PatternLayout layout = new PatternLayout("%m\r\n");
			Appender appender = new RollingFileAppender(layout, strLogFileName);
			cat.addAppender(appender);
			cat.setLevel(Level.DEBUG);
			logData = LogFactory.getLog(cat.getName());

			cat = repository.getLogger("pose.dataaq.log");
			cat.setAdditivity(false);
			layout = new PatternLayout("%m\r\n");
			appender = new RollingFileAppender(layout, strPoseLogFileName);
			cat.addAppender(appender);
			cat.setLevel(Level.DEBUG);
			logDataPoses = LogFactory.getLog(cat.getName());

		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}


	private long [] arrayLastTime = null;
	private long tLast = 0;
	private long tFirst = 0;


	public void runEngine()
	{
		bRunning = true;
		Thread thr = new Thread(this);
		thr.start();
	}


	public void stopEngine()
	{
		bRunning = false;
	}


	// Número de leitures completas observadas.
	private int sonar8Readins = 0;
	private long lastReading = 0;

	private boolean bRunning = false;

	public void run()
	{
	}
}
