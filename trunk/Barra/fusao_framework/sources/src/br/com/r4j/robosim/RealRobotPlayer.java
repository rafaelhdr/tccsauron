package br.com.r4j.robosim;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.ArrayList;
import java.util.StringTokenizer;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.realrobot.OdoSensor;
import br.com.r4j.robosim.realrobot.RealSonarSensor;


/**
 * Guia o robô conforme informação real.
 *
 *
 * arquivo de poses:
 *
 * pose:x(mm):y(mm):ang(graus):timeMillis
 *
 *
 * arquivo de sensores:
 *
 * s:sonarIdx:range:robot.x:robot.y:robot.theta:timeMillis
 * r:robot.x:robot.y:robot.theta:timeMillis
 * i:strImageName:x:robot.y:robot.theta:timeMillis
 *
 */
public class RealRobotPlayer extends RobotPlayer
{
	private static Log log = LogFactory.getLog(RealRobotPlayer.class.getName());

	private File flRealPose = null;
	private File flSensorData = null;

	private ArrayList listPosesReal = null;
	private ArrayList listPosesRealTimeMillis = null;

	private long [] times = null;

	// [idtTime][idxSonar]
	private int [][] sonarReadings = null;

	private Pose2D [] posesOdom = null;
//	private Pose2D [] posesReal = null;

	private int timeSize = 0;

	private OdoSensor odoSns = null;
	private RealSonarSensor sonSns = null;


	public RealRobotPlayer()
	{
		timeSize = 0;
	}


	public void setReadingsFile(File flSensorData) throws IOException
	{
		this.flSensorData = flSensorData;
		LineNumberReader reader = new LineNumberReader(new FileReader(flSensorData));
		try
		{
			ArrayList listLines = new ArrayList();
			for (String strLine = reader.readLine(); strLine != null; strLine = reader.readLine())
			{
				if (strLine.trim().equals("") ||  strLine.startsWith(";"))
					continue;
				listLines.add(strLine);
			}

			times = new long[listLines.size()];
			sonarReadings = new int[listLines.size()][8];
			posesOdom = new Pose2D[listLines.size()];
//			posesReal = new Pose2D[listLines.size()];

			// s:sonarIdx:range:robot.x:robot.y:robot.theta:timeMillis
			// r:robot.x:robot.y:robot.theta:timeMillis
			// i:strImageName:x:robot.y:robot.theta:timeMillis
			long lastTime = -1;
			int idxTime = -1;
			for (int idxLine = 0; idxLine < listLines.size(); idxLine++)
			{
				String strLine = (String) listLines.get(idxLine);
				StringTokenizer strToks = new StringTokenizer(strLine, ":");
				String strSonarNumber = null, strRange = null, strImgName = null;
				String strPre = strToks.nextToken();
				if (strPre.equals("s"))
				{
					strSonarNumber = strToks.nextToken();
					strRange = strToks.nextToken();
				}
				else if (strPre.equals("r"))
				{
					// tchãn!
				}
				else if (strPre.equals("i"))
				{
					strImgName = strToks.nextToken();
				}
				else
				{
					log.error("dado de sensor não encontrado: " + strLine);
					continue;
				}
				String strX = strToks.nextToken();
				String strY = strToks.nextToken();
				String strAng = strToks.nextToken();
				String strTimemillis = strToks.nextToken();
				
				int x = Integer.parseInt(strX);
				int y = Integer.parseInt(strY);
				int ang = Integer.parseInt(strAng);
				long timemillis = Long.parseLong(strTimemillis);

				if (lastTime != timemillis)
				{
					idxTime++;
					lastTime = timemillis;
					times[idxTime] = timemillis;
					Pose2D pose = new Pose2D(x, y, ang);
					posesOdom[idxTime] = pose;

					for (int j = 0; j < 8; j++)
						sonarReadings[idxTime][j] = -1;
				}

				if (strPre.equals("s"))
				{
					int idxSonar = Integer.parseInt(strSonarNumber);
					int range = Integer.parseInt(strRange);
					sonarReadings[idxTime][idxSonar] = range;
				}

//				posesReal = new Pose2D[listLines.size()];
			}
		}
		catch (NumberFormatException e)
		{
			log.error("erro", e);
		}
		reader.close();
	}


	public void setRealPoseFile(File flRealPose) throws IOException
	{
		this.flRealPose = flRealPose;
		listPosesReal = new ArrayList();
		listPosesRealTimeMillis = new ArrayList();
		LineNumberReader reader = new LineNumberReader(new FileReader(flRealPose));
		try
		{
			for (String strLine = reader.readLine(); strLine != null; strLine = reader.readLine())
			{
				if (strLine.trim().equals("") ||  strLine.startsWith(";"))
					continue;
				StringTokenizer strToks = new StringTokenizer(strLine, ":");
				String strPre = strToks.nextToken();
				String strX = strToks.nextToken();
				String strY = strToks.nextToken();
				String strAng = strToks.nextToken();
				String strTimemillis = strToks.nextToken();
				log.debug("strPre = " + strPre + ", strX = " + strX + ", strY = " + strY + ", strAng = " + strAng + ", strTimemillis = " + strTimemillis);

				int x = Integer.parseInt(strX);
				int y = Integer.parseInt(strY);
				int ang = Integer.parseInt(strAng);
				long timemillis = Long.parseLong(strTimemillis);

				Pose2D poseIt = new Pose2D(x, y, ang);
				listPosesReal.add(poseIt);
				listPosesRealTimeMillis.add(new Long(timemillis));
			}
		}
		catch (NumberFormatException e)
		{
			log.error("erro", e);
		}
		reader.close();
	}


	protected void gatherData()
	{
		odoSns.setReadings(posesOdom[currentStep]);
		sonSns.setReadings(sonarReadings[currentStep]);
	}


	public int getNumberOfSteps()
	{
		return timeSize;
	}


	public void sensorAdded(Sensor sns)
	{
		if (sns instanceof OdoSensor)
			odoSns = (OdoSensor) sns;
		else if (sns instanceof RealSonarSensor)
			sonSns = (RealSonarSensor) sns;
	}


	public void resetBuffers()
	{
		currentStep = 0;
	}
}

