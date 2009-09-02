package br.com.r4j.robosim;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import br.com.r4j.robosim.estimator.Sensor;

import br.com.r4j.image.operation.threebandpacked.*;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.image.sequence.featurematch.vline.*;
import br.com.r4j.research.*;
import br.com.r4j.research.vline.*;
import br.com.r4j.research.image.sequence.estimator.*;


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
	private WorldMap map = null;

	private Pose2D poseIni = null;

	private VLineMapSensor vlineMapSns = null;
	private VLineStateSensor vlineStateSns = null;


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
		this.map = map;
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
				while (ang > 360) ang = ang - 360;
				while (ang < 0) ang = ang + 360;

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
					while (currAng > 360) currAng = currAng - 360;
					while (currAng < 0) currAng = currAng + 360;
					
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


	private Pose2D poseLastImg = null;
	private Pose2D poseActual = null;
	
	protected void gatherData(boolean bEngine)
	{
		poseActual = (Pose2D) listPoses.get(currentStep);
		logSens.debug("-- it: " + currentStep);
		logSens.debug(poseActual);

		log.debug("-- it: " + currentStep);
		log.debug("poseActual: " + poseActual);
		
		poseActual = poseActual.add(poseIni);
	
		if (!bEngine)
		{
			Iterator itTrakers = listRobotTrack.iterator();
			while (itTrakers.hasNext())
			{
				EstimatorRenderer track = (EstimatorRenderer) itTrakers.next();
				track.newPose(poseActual);
			}
		}
		
		if ((poseLastImg == null || poseActual.dist2(poseLastImg) > 10000 || 
			Math.abs(poseActual.getTheta() - poseLastImg.getTheta()) > 10*Math.PI/180) && 
			(vlineMapSns != null || vlineStateSns != null))
		{
			int imgWidth = 320;
			int imgHeight = 240;
			int [] imgData = new int[imgHeight*imgWidth];

			CameraModel camModel = null;
			if (vlineStateSns != null)
				camModel = vlineStateSns.getCameraModel();
			else
				camModel = vlineMapSns.getCameraModel();

			this.createMapImage(poseActual, map, camModel, imgData, imgHeight, imgWidth);

			if (vlineMapSns != null)
				vlineMapSns.setReadings(imgData, imgWidth, imgHeight);
			if (vlineStateSns != null)
				vlineStateSns.setReadings(imgData, imgWidth, imgHeight);
			poseLastImg = poseActual;
		}
		else
		{
			if (vlineMapSns != null)
				vlineMapSns.setReadings(null, 1, 1);
			if (vlineStateSns != null)
				vlineStateSns.setReadings(null, 1, 1);
		}


		Iterator itSns = this.getSensors().iterator();
		while (itSns.hasNext())
		{
			Sensor sns = (Sensor) itSns.next();
//			log.debug("sns.getName(): " + sns.getName());
			if (sns instanceof RealPoseDependent)
				((RealPoseDependent) sns).setRealPose(poseActual);
		}
	}
	

	protected AbstractDoubleVector getActualRealPose()
	{
		if (poseActual != null)
			return poseActual.convert2vector(); 
		else
			return null;
	}
	


	public int getNumberOfSteps()
	{
		return listPoses.size();
	}


	public void sensorAdded(Sensor sns)
	{
		if (sns instanceof VLineMapSensor)
			vlineMapSns = (VLineMapSensor) sns;
		else if (sns instanceof VLineStateSensor)
			vlineStateSns = (VLineStateSensor) sns;
	}


	public void resetBuffers()
	{
		currentStep = 0;
	}


	public void addRobotTracker(EstimatorRenderer track)
	{
		listRobotTrack.add(track);
	}


	private void createMapImage(Pose2D pose, WorldMap map, CameraModel camModel, int [] imgData, int imgHeight, int imgWidth)
	{
/*
		RigidBodyTranformation2D trafo = new RigidBodyTranformation2D(pose);
		List listLines = map.getOrderedExpectedProjections(trafo, camModel, 1);

		log.debug("doing fake image: listLines = " + listLines);
		if (listLines.size() == 0)
			return;

		VLineRefProj lineBehind = null;
		VLineRefProj lineInFront = (VLineRefProj) listLines.get(0);
		int xIdx = 0;

		log.debug("lineInFront.getU() - lineInFront.getLineRef().arrayR.length/2 = " + (lineInFront.getU() - lineInFront.getLineRef().arrayR.length/2));
		
		// Faz até a primeira.
		for (; xIdx < (int) lineInFront.getU() - lineInFront.getLineRef().arrayR.length/2; xIdx++)
		{
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
			{
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(lineInFront.getLineRef().arrayR[0], 
																					lineInFront.getLineRef().arrayG[0], 
																					lineInFront.getLineRef().arrayB[0]);
			}
		}
		for (int count = 0; xIdx < imgWidth && xIdx < (int) lineInFront.getU() + lineInFront.getLineRef().arrayR.length/2; xIdx++, count++)
		{
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
			{
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(lineInFront.getLineRef().arrayR[count], 
																					lineInFront.getLineRef().arrayG[count], 
																					lineInFront.getLineRef().arrayB[count]);
			}
		}

		lineBehind = lineInFront;

		// Faz até entre a primeira e útima.
		for (int i = 1; i < listLines.size(); i++)
		{
			lineInFront = (VLineRefProj) listLines.get(i);

			int deadZone = (int) ((lineInFront.getU() - lineInFront.getLineRef().arrayR.length/2) - 
					              (lineBehind.getU() + lineBehind.getLineRef().arrayR.length/2));
			if (deadZone >= 0)
			{
				for (int count = 0; deadZone > 0 && xIdx < imgWidth && xIdx < (int) lineInFront.getU() - lineInFront.getLineRef().arrayR.length/2; xIdx++, count++)
				{
					int color = 
							ThreeBandPackedUtil.getPackedPixel(
										(lineInFront.getLineRef().arrayR[0]*(count + 1) + 
										lineBehind.getLineRef().arrayR[lineBehind.getLineRef().arrayR.length - 1]*(deadZone - count))/deadZone, 
										(lineInFront.getLineRef().arrayG[0]*(count + 1) + 
										lineBehind.getLineRef().arrayG[lineBehind.getLineRef().arrayR.length - 1]*(deadZone - count))/deadZone, 
										(lineInFront.getLineRef().arrayB[0]*(count + 1) + 
										lineBehind.getLineRef().arrayB[lineBehind.getLineRef().arrayR.length - 1]*(deadZone - count))/deadZone);

					for (int yIdx = 0; yIdx < imgHeight; yIdx++)
					{
						imgData[xIdx + yIdx*imgWidth] = color;
					}
				}
				for (int count = 0; xIdx < imgWidth && xIdx <= (int) lineInFront.getU() + lineInFront.getLineRef().arrayR.length/2; xIdx++, count++)
				{
					for (int yIdx = 0; yIdx < imgHeight; yIdx++)
					{
						imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(lineInFront.getLineRef().arrayR[count], 
																							lineInFront.getLineRef().arrayG[count], 
																							lineInFront.getLineRef().arrayB[count]);
					}
				}
			}
			else
			{
				log.debug("deadZone < 0: lineInFront = " + lineInFront);
			}

			lineBehind = lineInFront;
		}
	
		// Faz depois da útlima.
		for (; xIdx < imgWidth; xIdx++)
		{
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
			{
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(
														lineBehind.getLineRef().arrayR[lineBehind.getLineRef().arrayR.length - 1], 
														lineBehind.getLineRef().arrayG[lineBehind.getLineRef().arrayR.length - 1], 
														lineBehind.getLineRef().arrayB[lineBehind.getLineRef().arrayR.length - 1]);
			}
		}
/*/
		RigidBodyTranformation2D trafo = new RigidBodyTranformation2D(pose);
		List listLines = map.getOrderedExpectedProjections(trafo, camModel, 1);

//		log.debug("doing fake image: listLines = " + listLines);
		if (listLines.size() == 0)
			return;

		VLineRefProj lineBehind = null;
		VLineRefProj lineInFront = (VLineRefProj) listLines.get(0);
		int xIdx = 0;

		// Faz até a primeira.
		for (; xIdx < (int) lineInFront.getU(); xIdx++)
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(lineInFront.getLineRef().rLeft, 
																					lineInFront.getLineRef().gLeft, 
																					lineInFront.getLineRef().bLeft);
		for (int count = 0; xIdx < imgWidth && xIdx <= (int) lineInFront.getU() + 4; xIdx++, count++)
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(
																			(lineInFront.getLineRef().rRight), 
																			(lineInFront.getLineRef().gRight), 
																			(lineInFront.getLineRef().bRight));
		lineBehind = lineInFront;
//		log.debug("lineInFront.getU(): " + lineInFront.getU());
//		log.debug("lineInFront.getLineRef():left " + lineInFront.getLineRef().rLeft + ", " + lineInFront.getLineRef().gLeft + ", " + lineInFront.getLineRef().bLeft);
//		log.debug("lineInFront.getLineRef():right " + lineInFront.getLineRef().rRight + ", " + lineInFront.getLineRef().gRight + ", " + lineInFront.getLineRef().bRight);

		// Faz até entre a primeira e útima.
		for (int i = 1; i < listLines.size(); i++)
		{
			lineInFront = (VLineRefProj) listLines.get(i);
//			log.debug("lineBehind.getU(): " + lineBehind.getU());
//			log.debug("lineBehind.getLineRef():left " + lineBehind.getLineRef().rLeft + ", " + lineBehind.getLineRef().gLeft + ", " + lineBehind.getLineRef().bLeft);
//			log.debug("lineBehind.getLineRef():right " + lineBehind.getLineRef().rRight + ", " + lineBehind.getLineRef().gRight + ", " + lineBehind.getLineRef().bRight);

			double deadZone = (lineInFront.getU() - lineBehind.getU() - 9);
//			log.debug("deadZone: " + deadZone);
			if (deadZone >= 0)
			{
				for (double count = 0; count <= deadZone && deadZone > 0 && xIdx < imgWidth && xIdx < (int) lineInFront.getU(); xIdx++, count += 1)
				{
					int colorR = (int) ((lineInFront.getLineRef().rLeft*(count) + lineBehind.getLineRef().rRight*(deadZone - count))/deadZone);
					int colorG = (int) ((lineInFront.getLineRef().gLeft*(count) + lineBehind.getLineRef().gRight*(deadZone - count))/deadZone);
					int colorB = (int) ((lineInFront.getLineRef().bLeft*(count) + lineBehind.getLineRef().bRight*(deadZone - count))/deadZone);
					int color = 
							ThreeBandPackedUtil.getPackedPixel(
								colorR, 
								colorG, 
								colorB);
					for (int yIdx = 0; yIdx < imgHeight; yIdx++)
						imgData[xIdx + yIdx*imgWidth] = color;
//					log.debug(xIdx + " -- " + colorR + ", " + colorG + ", " + colorB + " :: " + color);
				}
				for (int count = 0; xIdx < imgWidth && xIdx <= (int) lineInFront.getU(); xIdx++, count++)
					for (int yIdx = 0; yIdx < imgHeight; yIdx++)
						imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(
																			(lineInFront.getLineRef().rLeft), 
																			(lineInFront.getLineRef().gLeft), 
																			(lineInFront.getLineRef().bLeft));
				for (int count = 0; xIdx < imgWidth && xIdx <= (int) lineInFront.getU() + 4; xIdx++, count++)
					for (int yIdx = 0; yIdx < imgHeight; yIdx++)
						imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(
																			(lineInFront.getLineRef().rRight), 
																			(lineInFront.getLineRef().gRight), 
																			(lineInFront.getLineRef().bRight));
			}
			else
				log.debug("deadZone < 0: lineInFront = " + lineInFront);

//			log.debug("lineInFront.getU(): " + lineInFront.getU());
//			log.debug("lineInFront.getLineRef():left " + lineInFront.getLineRef().rLeft + ", " + lineInFront.getLineRef().gLeft + ", " + lineInFront.getLineRef().bLeft);
//			log.debug("lineInFront.getLineRef():right " + lineInFront.getLineRef().rRight + ", " + lineInFront.getLineRef().gRight + ", " + lineInFront.getLineRef().bRight);
			lineBehind = lineInFront;
		}
	
		// Faz depois da útlima.
		for (; xIdx < imgWidth; xIdx++)
			for (int yIdx = 0; yIdx < imgHeight; yIdx++)
				imgData[xIdx + yIdx*imgWidth] = ThreeBandPackedUtil.getPackedPixel(
														lineBehind.getLineRef().rRight, 
														lineBehind.getLineRef().gRight, 
														lineBehind.getLineRef().bRight);
//*/
	}
}
