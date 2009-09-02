package br.com.r4j.jmr.test;

import java.io.IOException;
import java.rmi.RemoteException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.log4j.Appender;
import org.apache.log4j.Level;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import org.apache.log4j.PatternLayout;
import org.apache.log4j.RollingFileAppender;
import org.apache.log4j.spi.LoggerRepository;

import robots.Robot;
import robots.RobotHandler;
import robots.Sonar;
import robots.SonarReading;
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
public class ParaFrenteRespostaSonar implements Pose2DGrabber, Runnable
{
	private static Log logResp = LogFactory.getLog("ParaFrenteRespostaSonar");
	private static Log log = LogFactory.getLog("br.com.r4j.jmr.debug");

	private Log logData = null;
	private Log logDataPoses = null;
	private Robot myRobot = null;
	private boolean bRobotConn = false;

	private RobotDescription robot = null;
	private Pose2D poseEstimateTotalOld = null;
	private AbstractDoubleSquareMatrix poseEstimateTotalOldCovar = null;


	public Pose2D getMovement(long imageTimestamp)
	{
		try
		{
			if (myRobot != null)
			{
				logDataPoses.debug("r:(" + (int)myRobot.getState().pos.dX + "," + (int)myRobot.getState().pos.dY + "," + (int)(myRobot.getState().pos.dTh) + "):dt:" + imageTimestamp);

				double dx = 0;
				double dy = 0;
				double dTheta = 0;
				if (poseEstimateTotalOld != null)
				{
					dx = myRobot.getState().pos.dX - poseEstimateTotalOld.getX();
					dy = myRobot.getState().pos.dY - poseEstimateTotalOld.getY();
					dTheta = myRobot.getState().pos.dTh - poseEstimateTotalOld.getTheta();
				}
				return new Pose2D(dx, dy, dTheta);
			}
			else
			{
				log.debug("myRobot == nulll!");
				return new Pose2D(0, 0, 0);
			}
		}
		catch (RemoteException e)
		{
			e.printStackTrace();
			return null;
		}
	}


	public Pose2D getPose(long imageTimestamp)
	{
		try
		{
			if (myRobot != null)
			{
				logDataPoses.debug("r:(" + (int)myRobot.getState().pos.dX + "," + (int)myRobot.getState().pos.dY + "," + (int)(myRobot.getState().pos.dTh) + "):dt:" + imageTimestamp);

				AbstractDoubleSquareMatrix covarTotal = new DoubleSquareMatrix(3);
				AbstractDoubleSquareMatrix covarTotalOld = poseEstimateTotalOldCovar;
				double dx = 0;
				double dy = 0;
				double dTheta = 0;
				if (poseEstimateTotalOld != null)
				{
					dx = Math.abs(myRobot.getState().pos.dX - poseEstimateTotalOld.getX());
					dy = Math.abs(myRobot.getState().pos.dY - poseEstimateTotalOld.getY());
					dTheta = Math.abs(myRobot.getState().pos.dTh - poseEstimateTotalOld.getTheta());
				}
				covarTotal.setElement(0, 0, covarTotalOld.getElement(0, 0) + dx*robot.getDXVariancePerMilimiter());
				covarTotal.setElement(1, 1, covarTotalOld.getElement(1, 1) + dy*robot.getDYVariancePerMilimiter());
				covarTotal.setElement(2, 2, covarTotalOld.getElement(2, 2) + dTheta*robot.getDThetaVariancePerRad());
				poseEstimateTotalOld = new Pose2D(myRobot.getState().pos.dX, myRobot.getState().pos.dY, myRobot.getState().pos.dTh);
				poseEstimateTotalOldCovar = covarTotal;
				return poseEstimateTotalOld;
			}
			else
			{
				log.debug("myRobot == nulll!");
				return new Pose2D(0, 0, 0);
			}
		}
		catch (RemoteException e)
		{
			e.printStackTrace();
			return null;
		}
	}


	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp)
	{
		return null;
	}


	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp)
	{
		try
		{
			AbstractDoubleSquareMatrix covarInc = new DoubleSquareMatrix(3);
			if (myRobot != null)
			{
				double dx = 0;
				double dy = 0;
				double dTheta = 0;
				if (poseEstimateTotalOld != null)
				{
					dx = myRobot.getState().pos.dX - poseEstimateTotalOld.getX();
					dy = myRobot.getState().pos.dY - poseEstimateTotalOld.getY();
					dTheta = myRobot.getState().pos.dTh - poseEstimateTotalOld.getTheta();
				}
				covarInc.setElement(0, 0, Math.abs(dx)*robot.getDXVariancePerMilimiter());
				covarInc.setElement(1, 1, Math.abs(dy)*robot.getDYVariancePerMilimiter());
				covarInc.setElement(2, 2, Math.abs(dTheta)*robot.getDThetaVariancePerRad());
			}
			return covarInc;
		}
		catch (RemoteException e)
		{
			e.printStackTrace();
			return null;
		}
	}


	public void initEngine()
	{
		robot = new RobotDescription();
		try
		{
			PropertiesHolder props = br.com.r4j.configurator.Configurator.getPropsHolder();
			bRobotConn = Integer.parseInt(props.getStringProperty("odo-acq/robot-connect")) != 0 || props.getStringProperty("odo-acq/robot-connect").equalsIgnoreCase("true");

			String strLogName = props.getStringProperty("odo-acq/log-file-name");
			if (!strLogName.startsWith("/"))
				strLogName = "/" + strLogName;
			String strLogFileName = props.getStringProperty("log4j/log4j-logs-location") + strLogName;
			strLogName = props.getStringProperty("odo-acq/pose-log-file-name");
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

			String strConfPioneerFileName = props.getStringProperty("odo-acq/pioneer-conf");
			if (bRobotConn)
				myRobot = RobotHandler.connectToRobot("localhost", 6688, RobotHandler.iSAPHIRA_COM1, strConfPioneerFileName);
			else
				myRobot = RobotHandler.connectToRobot("localhost", 6688, RobotHandler.iSAPHIRA_SIM_LOCAL, strConfPioneerFileName);

			log.debug("myRobot = " + myRobot.getClass().getName());
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
		try
		{
			// inicia algumas variaveis.
			arrayLastTime = new long[8];
			tLast = System.currentTimeMillis(); tFirst = tLast;
			for (int i = 0; i < 8; i++)
			{
				Sonar sonar = myRobot.getSonar(i);
				if (sonar != null)
					log.debug("s"+i+"_desc:(" + (int)sonar.getPosition2D().dX + "," + (int)sonar.getPosition2D().dY + "," + (int)(sonar.getPosition2D().dTh) + ")");
			}
			for (int i = 0; i < 8; i++)
				arrayLastTime[i] = System.currentTimeMillis();

			do
			{
				this.extractReadings();
			}
			while (sonar8Readins <= 1);

			myRobot.setVelocity(50);

			long lastSmallerReading = 9999;
			do
			{
				this.extractReadings();
				if (!bRunning)
				{
					log.debug("saindo ...");
					RobotHandler.disconnectFromRobot("localhost", 6688, myRobot);
					return;
				}
			}
			while (lastSmallerReading > 30);

			myRobot.setVelocity(0);

			int base_sonar8Readins = sonar8Readins;
			do
			{
				this.extractReadings();
			}
			while ((sonar8Readins - base_sonar8Readins) <= 2);

			RobotHandler.disconnectFromRobot("localhost", 6688, myRobot);
		}
		catch (Exception e)
		{
			RobotHandler.disconnectFromRobot("localhost", 6688, myRobot);
			e.printStackTrace();
		}
	}


	public void goRight()
	{
		log.debug("goRight()!");
		try
		{
			if (myRobot != null)
			{
				double dAng = 5 + 5*Math.random();
				myRobot.setDHeading(-dAng);
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}


	public void goLeft()
	{
		log.debug("goRight()!");
		try
		{
			if (myRobot != null)
			{
				double dAng = 5 + 5*Math.random();
				myRobot.setDHeading(dAng);
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}


	public void speed()
	{
		log.debug("speed()!");
		try
		{
			if (myRobot != null)
			{
				double speed = myRobot.getState().dLVel + 10*Math.random();
				myRobot.setVelocity(speed);
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}


	public void stop()
	{
		log.debug("stop()!");
		try
		{
			if (myRobot != null)
			{
				double speed = myRobot.getState().dLVel - 50*Math.random();
				speed = 0;
				myRobot.setVelocity(speed);
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}

	private void extractReadings() throws InterruptedException, RemoteException
	{
		long t = System.currentTimeMillis();
		long dt = t - tFirst;
		tLast = t;
		long actualReading = lastReading;
		logData.debug("r:(" + (int)myRobot.getState().pos.dX + "," + (int)myRobot.getState().pos.dY + "," + (int)(myRobot.getState().pos.dTh) + "):dt:" + dt);

		for (int i = 0; i < 8; i++)
		{
			Sonar sonar = myRobot.getSonar(i);
			if (sonar != null && sonar.getLastReading() != null)
			{
				SonarReading sonarReading = sonar.getLastReading();
				if (sonarReading.iIdent > lastReading)
				{
					dt = t - tFirst;
					arrayLastTime[i] = t;
					if (sonarReading.iIdent > actualReading)
						actualReading = sonarReading.iIdent;
					if (sonarReading.pos != null)
					{
						logData.debug("s:" + i + ":r:" + (int)sonarReading.dRange + ":(" + (int)sonarReading.pos.dX + "," + (int)sonarReading.pos.dY  + "," + (int)(180*sonarReading.pos.dTh/Math.PI) + "):dt:" + dt);
						sonar8Readins += (i == 7) ? 1 : 0;
					}
				}
			}
		}
		lastReading = actualReading;
		Thread.currentThread().sleep(20);
	}
}
