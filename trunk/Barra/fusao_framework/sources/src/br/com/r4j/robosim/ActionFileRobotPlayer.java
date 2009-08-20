package br.com.r4j.robosim;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.StringTokenizer;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import br.com.r4j.robosim.estimator.Sensor;


/**
 * Guia o robô através de um conjunto de regras qeu ditam o movimento do robô.
 *
 *
 * arquivo de actions:
 *
 * mov:distancia(mm):velocidade(mm/it):angulo(graus):velocidade(graus/it)
 *
 */
public class ActionFileRobotPlayer extends RobotPlayer implements RealTrackGenerator, MapDependent
{
	private static Log log = LogFactory.getLog(ActionFileRobotPlayer.class.getName());
	private static Log logSens = LogFactory.getLog("realpose");

	private File flAct = null;
	private ArrayList listPoses = null;
	private ArrayList listRobotTrack = null;

	private Pose2D poseIni = null;


	public ActionFileRobotPlayer()
	{
		listRobotTrack = new ArrayList();
		listPoses = new ArrayList();
		currentStep = 0;
	}


	public void setWorldMap(WorldMap map)
	{
		AbstractDoubleVector loc = map.getInitialLocalization();
		poseIni = new Pose2D(loc.getComponent(0), loc.getComponent(1), loc.getComponent(2));
		log.debug("setWorldMap:poseIni: " + poseIni);
	}


	public void setActionFile(File flAct) throws IOException
	{
		currentStep = 0;
		this.flAct = flAct;
		listPoses.clear();
		Pose2D poseRobotIni = new Pose2D(0, 0, 0);
//		poseRobotIni.setPose(0, 0, 0);
		listPoses.add(poseRobotIni);
		LineNumberReader reader = new LineNumberReader(new FileReader(flAct));
		try
		{
			Pose2D poseItPast = poseRobotIni;
			for (String strLine = reader.readLine(); strLine != null; strLine = reader.readLine())
			{
				if (strLine.trim().equals("") ||  strLine.startsWith(";"))
					continue;
				StringTokenizer strToks = new StringTokenizer(strLine, ":");
				String strPre = strToks.nextToken();
				String strDist = strToks.nextToken();
				String strVel = strToks.nextToken();
				String strAng = strToks.nextToken();
				String strVelAng = strToks.nextToken();
				log.debug("strPre = " + strPre + ", strDist = " + strDist + ", strVel = " + strVel + ", strAng = " + strAng + ", strVelAng = " + strVelAng);

				int dist = Integer.parseInt(strDist);
				int vel = Integer.parseInt(strVel);
				int ang = Integer.parseInt(strAng);
				int velAng = Integer.parseInt(strVelAng);
/*				
				if ((ang < 0 && velAng >= 0) || (ang > 0 && velAng <= 0))
				{
					velAng -= velAng; 
					log.debug("\tcorrection: ang = " + ang + ", velAng = " + velAng);
				}
//*/
				while (ang > 360)
					ang = ang - 360;
				while (ang < 0)
					ang = ang + 360;

				int currDist = 0, currAng = 0;
				while (currDist < dist || Math.abs(currAng - ang) > Math.abs(velAng))
				{
					float stepDist = 0, stepAng = 0;
					if (currDist < dist)
					{
						stepDist = vel;
						currDist += vel;
					}
					if (Math.abs(currAng - ang) > Math.abs(velAng))
					{
						stepAng = velAng;
						currAng += velAng;
					}
					while (currAng > 360)
						currAng = currAng - 360;
					while (currAng < 0)
						currAng = currAng + 360;
					
					Pose2D poseIt = new Pose2D(poseItPast.getX() + Math.cos(poseItPast.getTheta())*stepDist, 
								poseItPast.getY() + Math.sin(poseItPast.getTheta())*stepDist, 
								poseItPast.getTheta() + stepAng*Math.PI/180);
					listPoses.add(poseIt);
					poseItPast = poseIt;
				}
				
				if (Math.abs(currAng - ang) > 0)
				{
					float stepDist = 0, stepAng = ang - currAng;
					Pose2D poseIt = new Pose2D(poseItPast.getX() + Math.cos(poseItPast.getTheta())*stepDist, 
								poseItPast.getY() + Math.sin(poseItPast.getTheta())*stepDist, 
								poseItPast.getTheta() + stepAng*Math.PI/180);
					listPoses.add(poseIt);
					poseItPast = poseIt;
				}

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
		Pose2D poseActual = (Pose2D) listPoses.get(currentStep);
		logSens.debug("-- it: " + currentStep);
		logSens.debug(poseActual);
		
		poseActual = poseActual.add(poseIni);
	
//		log.debug("gatherData: " + poseActual);
		
		Iterator itTrakers = listRobotTrack.iterator();
		while (itTrakers.hasNext())
		{
			EstimatorRenderer track = (EstimatorRenderer) itTrakers.next();
			track.newPose(poseActual);
		}
		
//		log.debug("this.getSensors().size(): " + this.getSensors().size());
		Iterator itSns = this.getSensors().iterator();
		while (itSns.hasNext())
		{
			Sensor sns = (Sensor) itSns.next();
//			log.debug("sns.getName(): " + sns.getName());
			if (sns instanceof RealPoseDependent)
				((RealPoseDependent) sns).setRealPose(poseActual);
		}
	}


	public int getNumberOfSteps()
	{
		return listPoses.size();
	}


	public void sensorAdded(Sensor sns)
	{
		// acho que nada precisa ser feito ...
	}


	public void resetBuffers()
	{
		currentStep = 0;
	}


	public void addRobotTracker(EstimatorRenderer track)
	{
		listRobotTrack.add(track);
	}
}
