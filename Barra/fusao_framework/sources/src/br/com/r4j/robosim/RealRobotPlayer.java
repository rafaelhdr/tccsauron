package br.com.r4j.robosim;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.StringTokenizer;
import java.text.*;
import java.awt.image.BufferedImage;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.research.image.sequence.estimator.*;
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
public class RealRobotPlayer extends RobotPlayer implements RealTrackGenerator, MapDependent
{
	private static Log log = LogFactory.getLog(RealRobotPlayer.class.getName());
	private static Log logStats = LogFactory.getLog("estatisticas");

	private File flRealPose = null;
	private File flSensorData = null;

	private ArrayList listRobotTrack = null;

	private long [] times = null;
//	private long [] timesReal = null;

	// [idtTime][idxSonar]
	private int [][] sonarReadings = null;
	
	private File [] vlineReadings = null;

	private Pose2D [] posesOdom = null;
	private Pose2D [] posesReal = null;

	private double [] distances = null;
	private double [] realIdx = null;
	private int [] odomIdx = null;

	private Pose2D poseIni = null;

	private int timeSize = 0;

	private int idxRelOffset = 0;

	private OdoSensor odoSns = null;
	private RealSonarSensor sonSns = null;
	private VLineMapSensor vlineMapSns = null;
	private VLineStateSensor vlineStateSns = null;

	private NumberFormat nFrmt = NumberFormat.getInstance();

	int imgCountProcessed = 0;
	int currentStepImgLast = -1;
	double dImgMean = 0;

	int odomCountProcessed = 0;
	int currentStepOdomLast = -1;
	double dOdomMean = 0;

	int [] sonCountProcessed = new int[8];
	int [] currentStepSonLast = new int[8];
	double [] dSonMean = new double[8];


	public RealRobotPlayer()
	{
		timeSize = 0;
		listRobotTrack = new ArrayList();
		nFrmt.setMaximumFractionDigits(3);

		for (int j = 0; j < 8; j++)
		{
			sonCountProcessed[j] = 0;
			currentStepSonLast[j] = -1;
			dSonMean[j] = 0;
		}
	}


	public void setWorldMap(WorldMap map)
	{
		AbstractDoubleVector loc = map.getInitialLocalization();
		poseIni = new Pose2D(loc.getComponent(0), loc.getComponent(1), loc.getComponent(2));
		log.debug("setWorldMap:poseIni: " + poseIni);
	}


	public void setReadingsFile(File flSensorData, File flRealPose) throws IOException
	{
		this.flSensorData = flSensorData;
		this.flRealPose = flRealPose;

		LineNumberReader readerReal = null;
		if (flRealPose != null)
			readerReal = new LineNumberReader(new FileReader(flRealPose));

		LineNumberReader reader = new LineNumberReader(new FileReader(flSensorData));
		try
		{
			ArrayList listLinesReal = new ArrayList();
			if (readerReal != null) for (String strLine = readerReal.readLine(); strLine != null; strLine = readerReal.readLine())
			{
				if (strLine.trim().equals("") ||  strLine.startsWith(";"))
					continue;
				listLinesReal.add(strLine);
			}

			ArrayList listLines = new ArrayList();
			for (String strLine = reader.readLine(); strLine != null; strLine = reader.readLine())
			{
				if (strLine.trim().equals("") ||  strLine.startsWith(";"))
					continue;
				listLines.add(strLine);
			}
			log.debug("listLines.size(): " + listLines.size() + ", listLinesReal.size(): " + listLinesReal.size());

			times = new long[listLines.size()];
			sonarReadings = new int[listLines.size()][8];
			vlineReadings = new File[listLines.size()];
			posesOdom = new Pose2D[listLines.size()];
			distances = new double[listLines.size()];
			realIdx = new double[listLines.size()];
			odomIdx = new int[listLines.size()];
			odomIdx[0] = 1;

//			timesReal = new long[listLinesReal.size()];
//			posesReal = new Pose2D[listLinesReal.size()];
//			timesReal = new long[listLines.size()];
			posesReal = new Pose2D[listLines.size()];

			// s:sonarIdx:range:robot.x:robot.y:robot.theta:timeMillis
			// r:robot.x:robot.y:robot.theta:timeMillis
			// i:strImageName:x:robot.y:robot.theta:timeMillis
			Pose2D poseLastOdom = null, poseLastSon = null, poseLastImg = null;
			int nextSonarIdx = 0;
			long lastTime = -1; int idxTime = -1; boolean bGetOdomWithinLimit = false;
			int countOdomOnly = 0;
			for (int idxLine = 0; idxLine < listLines.size(); idxLine++)
			{
				String strLine = (String) listLines.get(idxLine);
				StringTokenizer strToks = new StringTokenizer(strLine, ",:");
				String strSonarNumber = null, strRange = null, strImgName = null;
				String strPre = strToks.nextToken();
				if (strPre.equals("s"))
					{strSonarNumber = strToks.nextToken(); strRange = strToks.nextToken();}
				else if (strPre.equals("i"))
					strImgName = strToks.nextToken().replace(';', ':');
				else if (!strPre.equals("r"))
					{log.error("dado de sensor não encontrado: " + strLine); continue;}

				int x = Integer.parseInt(strToks.nextToken()), y = Integer.parseInt(strToks.nextToken());
				int ang = Integer.parseInt(strToks.nextToken());
				long timemillis = Long.parseLong(strToks.nextToken());
//				Pose2D pose = new Pose2D(x, y, ang*1.0*Math.PI/180.0);
				Pose2D pose = new Pose2D(x, -y, (360-ang)*1.0*Math.PI/180.0);

				// Rejeição
				int d = 10000;
				double dang = 1000;
				int idxSonar = -10;
				if (strPre.equals("s"))
				{
					if (poseLastSon != null)
						{d = (int) poseLastSon.getDistance(x, -y); dang = poseLastSon.getAngularDistance((360-ang)*1.0*Math.PI/180.0);}
					if (d < 20 && dang < 5.0*Math.PI/180)
						continue;
					else
					{
						idxSonar = Integer.parseInt(strSonarNumber);
						// Coxa Bamba!
						idxSonar = 7 - idxSonar;

						if (nextSonarIdx == idxSonar)
						{
							nextSonarIdx++; 
							nextSonarIdx = nextSonarIdx%8;
							poseLastSon = pose;
							bGetOdomWithinLimit = false;
						}
						else
						{
							if (poseLastOdom == null)
								poseLastOdom = pose;
							d = (int) poseLastOdom.getDistance(pose); dang = poseLastOdom.getAngularDistance(pose);
							if (d > 50 || dang > 5.0*Math.PI/180)
								bGetOdomWithinLimit = false;
							continue;
						}
					}
				}
				else if (strPre.equals("i"))
				{
					if (poseLastImg != null)
						{d = (int) poseLastImg.getDistance(pose);dang = poseLastImg.getAngularDistance(pose);}
					if (d < 400 && dang < 10.0*Math.PI/180)
//					if (d < 200 && dang < 5.0*Math.PI/180)
//					if (d < 300 && dang < 15.0*Math.PI/180)
					{
						if (poseLastOdom == null)
							poseLastOdom = pose;
						d = (int) poseLastOdom.getDistance(pose); dang = poseLastOdom.getAngularDistance(pose);
						if (d > 50 || dang > 5.0*Math.PI/180)
							bGetOdomWithinLimit = false;
						continue;
					}
					else
					{
						bGetOdomWithinLimit = false;
						poseLastImg = pose;
					}
				}
				else if (strPre.equals("r"))
				{
					if (poseLastOdom != null)
						{d = (int) poseLastOdom.getDistance(pose); dang = poseLastOdom.getAngularDistance(pose);}
					if (!bGetOdomWithinLimit && d < 50 && dang < 5.0*Math.PI/180)
						continue;
					else
						poseLastOdom = pose;
				}

				if (lastTime != timemillis)
				{
					idxTime++;
					lastTime = timemillis;
					times[idxTime] = timemillis;
					
					posesOdom[idxTime] = pose;
					if (idxTime > 0)
						distances[idxTime] = posesOdom[idxTime].getDistance(posesOdom[idxTime - 1]);
					else
						distances[idxTime] = 0;

					for (int j = 0; j < 8; j++)
						sonarReadings[idxTime][j] = -1;
//					log.debug("pobs("+strPre+", "+idxSonar+", "+nextSonarIdx+", "+d+", "+dang+"): " + pose + ", timemillis: " + timemillis);
				}

				if (strPre.equals("s"))
				{
					idxSonar = Integer.parseInt(strSonarNumber);

					// Coxa Bamba!
					idxSonar = 7 - idxSonar;

					int range = Integer.parseInt(strRange);
					sonarReadings[idxTime][idxSonar] = range;
				}
				if (strPre.equals("i"))
				{
					vlineReadings[idxTime] = new File(strImgName); 
				}

				if (idxTime > 0)
				{
					if (strPre.equals("r"))
						odomIdx[idxTime] = odomIdx[idxTime - 1] + 1;
					else
						odomIdx[idxTime] = odomIdx[idxTime - 1];
				}
			}
			log.debug("obs:timeSize: " + timeSize);
			timeSize = idxTime;

			if (readerReal != null) 
			{
				Pose2D poseIni = null;

				boolean bRefFix = false;
				int idxLineLastRefFix = 0;
				for (int idxLine = 0; idxLine < listLinesReal.size(); idxLine++)
				{
					String strLine = (String) listLinesReal.get(idxLine);
					StringTokenizer strToks = new StringTokenizer(strLine, ",:"); String strPre = strToks.nextToken();
					if (!strPre.equals("r"))					
						{log.error("dado de sensor não encontrado: " + strLine); continue;}

					int x = Integer.parseInt(strToks.nextToken()), y = -Integer.parseInt(strToks.nextToken());
					int ang = Integer.parseInt(strToks.nextToken());
					long timemillis = Long.parseLong(strToks.nextToken());
					int idxMatch = Integer.parseInt(strToks.nextToken());

					if (idxMatch > 0)
					{
						bRefFix = true;
						idxLineLastRefFix = idxLine;
					}
				}

				int idxLineFirst = 0;
				log.debug("bRefFix = " + bRefFix + ", idxLineLastRefFix: " + idxLineLastRefFix);
				int idxLast = 0, idxLastOdom = 0;
				if (bRefFix)
				{
					idxLineFirst = idxLineLastRefFix;
					idxTime = 0;
					Pose2D [] arrayPoseReal = new Pose2D[listLinesReal.size()];
					for (int idxLine = 0; idxLine < idxLineLastRefFix + 1; idxLine++)
					{
						String strLine = (String) listLinesReal.get(idxLine);
						StringTokenizer strToks = new StringTokenizer(strLine, ",:"); String strPre = strToks.nextToken();

						int x = Integer.parseInt(strToks.nextToken()), y = -Integer.parseInt(strToks.nextToken());
						int ang = Integer.parseInt(strToks.nextToken());
						long timemillis = Long.parseLong(strToks.nextToken());
						int idxMatch = Integer.parseInt(strToks.nextToken());
						Pose2D pose = new Pose2D(x, y, 0);

						if (idxTime == 0)
						{
							poseIni = pose;
							pose = pose.sub(poseIni);
							realIdx[idxTime] = (idxLine+1);

							posesReal[idxTime++] = pose;
							log.debug("posesReal["+(idxTime-1)+"].getX(): " + posesReal[(idxTime-1)].getX() + ", posesReal["+(idxTime-1)+"].getY(): " + posesReal[(idxTime-1)].getY());

							arrayPoseReal[idxLine] = pose;
						}
						else if (idxMatch <= 0)
						{
							pose = pose.sub(poseIni);
							arrayPoseReal[idxLine] = pose;
						}
						else if (idxMatch > 0)
						{
							log.debug("idxMatch: " + idxMatch + ", idxTime: " + idxTime + ", idxLastOdom: " + idxLastOdom + ", idxLast: " + idxLast + ", idxLine: " + idxLine);
							pose = pose.sub(poseIni);
							arrayPoseReal[idxLine] = pose;

							double dTotalOdom = 0; for (int i = idxLastOdom + 1; i <= idxMatch; i++)
								dTotalOdom += posesOdom[i].getDistance(posesOdom[i - 1]);
							double dTotalReal = 0; for (int i = idxLast + 1; i <= idxLine; i++)
								dTotalReal += arrayPoseReal[i].getDistance(arrayPoseReal[i - 1]);
							double dTotalRealX = 0; for (int i = idxLast + 1; i <= idxLine; i++)
								dTotalRealX += arrayPoseReal[i].getX() - arrayPoseReal[i - 1].getX();
							double dTotalRealY = 0; for (int i = idxLast + 1; i <= idxLine; i++)
								dTotalRealY += arrayPoseReal[i].getY() - arrayPoseReal[i - 1].getY();
							double dTotalOdomDebug = 0;
							double dOdomSobra = 0;
							int countCount = 0;
							int iOdom = idxLastOdom + 1, idxLastOdomInt = idxLastOdom;
							for (int iReal = idxLast + 1; iReal <= idxLine; iReal++)
							{
								double dReal = arrayPoseReal[iReal].getDistance(arrayPoseReal[iReal - 1]);
								double dRealX = arrayPoseReal[iReal].getX() - arrayPoseReal[iReal - 1].getX();
								double dRealY = arrayPoseReal[iReal].getY() - arrayPoseReal[iReal - 1].getY();
								log.debug("dReal: " + dReal);
								if (dReal > 0)
								{
									double dOdomIdeal = dReal / (dTotalReal / dTotalOdom);
									double dRealRateX = (dRealX / dReal) * (dTotalReal / dTotalOdom);
									double dRealRateY = (dRealY / dReal) * (dTotalReal / dTotalOdom);
									log.debug("arrayPoseReal[iReal]: " + arrayPoseReal[iReal] + ", arrayPoseReal[iReal - 1]: " + arrayPoseReal[iReal - 1]);
									log.debug("idxLastOdomInt: " + idxLastOdomInt + ", idxMatch: " + idxMatch + ", iReal: " + iReal + ", iOdom: " + iOdom);
									log.debug("dReal: " + dReal + ", dOdomIdeal: " + dOdomIdeal + ", dRealRateX: " + dRealRateX + ", dOdomSobra: " + dOdomSobra);
									double dOdomAcc = 0, dOdomAccX = 0, dOdomAccY = 0;
									for (; iOdom <= idxMatch && dOdomAcc < dOdomIdeal; iOdom++)
									{
										countCount++;
										dOdomAcc = posesOdom[iOdom].getDistance(posesOdom[idxLastOdomInt]);
										dOdomAccX = Math.abs(posesOdom[iOdom].getX() - posesOdom[idxLastOdomInt].getX());
										dOdomAccY = Math.abs(posesOdom[iOdom].getY() - posesOdom[idxLastOdomInt].getY());
										log.debug("dOdomAccX: " + dOdomAccX + ", dOdomAccY: " + dOdomAccY);

										double rrateYX = 0, rrateXY = 0;
										if (Math.abs(dRealRateX) > 0.01) rrateYX = dRealRateY/dRealRateX;
										else rrateYX = 100000;
										if (Math.abs(dRealRateY) > 0.01) rrateXY = dRealRateX/dRealRateY;
										else rrateXY = 100000;
										double ddX = dOdomAcc/Math.sqrt(1 + rrateYX*rrateYX), ddY = dOdomAcc/Math.sqrt(1 + rrateXY*rrateXY);
										if (Math.abs(dRealRateX) > 0.01) ddX = (dRealRateX/Math.abs(dRealRateX))*ddX;
										if (Math.abs(dRealRateY) > 0.01) ddY = (dRealRateY/Math.abs(dRealRateY))*ddY;
										posesReal[iOdom] = new Pose2D(posesReal[idxLastOdomInt].getX() + ddX, 
																	  posesReal[idxLastOdomInt].getY() + ddY, 0);
										double dx = posesReal[iOdom].getX() - posesReal[iOdom - 1].getX(),
											   dy = posesReal[iOdom].getY() - posesReal[iOdom - 1].getY();
										posesReal[iOdom].setTheta(FunctionsR.angle(dx, dy));

										realIdx[iOdom] = realIdx[idxLast] + ((iOdom - idxLast)*1.0)/(1.0*(idxMatch - idxLast))*(idxLine-idxLast);
										log.debug("posesReal["+(iOdom)+"].getX(): " + posesReal[(iOdom)].getX() + ", posesReal["+(iOdom)+"].getY(): " + posesReal[(iOdom)].getY());
										log.debug("realIdx["+(iOdom)+"]: " + realIdx[(iOdom)]);
									}
									dTotalOdomDebug += dOdomAcc;
									log.debug("dTotalOdomDebug: " + dTotalOdomDebug + ", dTotalOdom: " + dTotalOdom);
									if (Math.abs(posesReal[iOdom-1].getX() - arrayPoseReal[iReal].getX()) > Math.abs(posesReal[iOdom-1].getY() - arrayPoseReal[iReal].getY()))
									{
										if ((posesReal[iOdom-1].getX() - arrayPoseReal[iReal].getX()) > 0)
											dOdomSobra = -posesReal[iOdom-1].getDistance(arrayPoseReal[iReal]);
										else
											dOdomSobra = posesReal[iOdom-1].getDistance(arrayPoseReal[iReal]);
									}
									else
									{
										if ((posesReal[iOdom-1].getY() - arrayPoseReal[iReal].getY()) > 0)
											dOdomSobra = -posesReal[iOdom-1].getDistance(arrayPoseReal[iReal]);
										else
											dOdomSobra = posesReal[iOdom-1].getDistance(arrayPoseReal[iReal]);
									}

									double dx = posesReal[iOdom-1].getX() - posesReal[iOdom-2].getX(),
										   dy = posesReal[iOdom-1].getY() - posesReal[iOdom-2].getY();
									posesReal[iOdom-1].setTheta(FunctionsR.angle(dx, dy));
/*
									realIdx[iOdom] = iReal;
									idxLastOdomInt = iOdom;
//*/
									realIdx[iOdom-1] = iReal;
									idxLastOdomInt = iOdom-1;
									log.debug("-("+dOdomSobra+")--posesReal["+(iOdom-1)+"].getX(): " + posesReal[(iOdom-1)].getX() + ", posesReal["+(iOdom-1)+"].getY(): " + posesReal[(iOdom-1)].getY());
									/*
									for (int iii = 0; iii < countCount; iii++)
									{
										double rrateYX = 0, rrateXY = 0;
										if (Math.abs(dRealRateX) > 0.01) rrateYX = dRealRateY/dRealRateX;
										else rrateYX = 100000;
										if (Math.abs(dRealRateY) > 0.01) rrateXY = dRealRateX/dRealRateY;
										else rrateXY = 100000;
										double ddX = (dOdomSobra/Math.sqrt(1 + rrateYX*rrateYX))*(iii+1.0)/(1.0*countCount), 
											   ddY = (dOdomSobra/Math.sqrt(1 + rrateXY*rrateXY))*(iii+1.0)/(1.0*countCount);
										if (Math.abs(dRealRateX) > 0.01) ddX = (dRealRateX/Math.abs(dRealRateX))*ddX;
										if (Math.abs(dRealRateY) > 0.01) ddY = (dRealRateY/Math.abs(dRealRateY))*ddY;
//										double ddX = (dRealRateX/Math.abs(dRealRateX))*dOdomSobra/Math.sqrt(1 + rrateYX*rrateYX)*(iii+1.0)/(1.0*countCount);
										log.debug("(antes)     posesReal["+(iOdom - countCount + iii)+"].getX(): " + posesReal[(iOdom - countCount + iii)].getX() + ", posesReal["+(iOdom - countCount + iii)+"].getY(): " + posesReal[(iOdom - countCount + iii)].getY());
										posesReal[iOdom - countCount + iii] = new Pose2D(posesReal[iOdom - countCount + iii].getX() + ddX, 
																	posesReal[iOdom - countCount + iii].getY() + ddY, 
																	posesReal[iOdom - countCount + iii].getTheta());
										log.debug("(corrigido) posesReal["+(iOdom - countCount + iii)+"].getX(): " + posesReal[(iOdom - countCount + iii)].getX() + ", posesReal["+(iOdom - countCount + iii)+"].getY(): " + posesReal[(iOdom - countCount + iii)].getY());
										log.debug("iii: "+iii+", ddX: "+ddX+", countCount: " + countCount + ", dRealRateX: "+dRealRateX+", rrateYX: " + rrateYX);
									}
									dOdomSobra = 0; countCount = 0;
									//*/
//									posesReal[iOdom-1] = arrayPoseReal[iReal];
									log.debug("-("+dOdomSobra+")--posesReal["+(iOdom-1)+"].getX(): " + posesReal[(iOdom-1)].getX() + ", posesReal["+(iOdom-1)+"].getY(): " + posesReal[(iOdom-1)].getY());
									log.debug("-("+dOdomSobra+")--arrayPoseReal["+iReal+"].getX(): " + arrayPoseReal[iReal].getX() + ", arrayPoseReal["+iReal+"].getY(): " + arrayPoseReal[iReal].getY());
								}
								else
								{
									for (; iOdom <= idxMatch; iOdom++)
									{
										posesReal[iOdom] = arrayPoseReal[iReal];
										posesReal[iOdom].setTheta(posesOdom[iOdom].getTheta());
										realIdx[iOdom] = realIdx[idxLast] + ((iOdom - idxLast)*1.0)/(1.0*(idxMatch - idxLast));
										log.debug("posesReal["+(iOdom)+"].getX(): " + posesReal[(iOdom)].getX() + ", posesReal["+(iOdom)+"].getY(): " + posesReal[(iOdom)].getY());
									}
									realIdx[iOdom] = iReal;
									idxLastOdomInt = iOdom-1;
								}
							}

							double dRealRateX = (dTotalRealX / dTotalReal) * (dTotalReal / dTotalOdom);
							double dRealRateY = (dTotalRealY / dTotalReal) * (dTotalReal / dTotalOdom);
							log.debug("dOdomSobra: " + dOdomSobra + ", " + posesReal + ", " + dRealRateX + ", " + dRealRateY);
							for (int iii = 0; iii < countCount; iii++)
							{
								double rrateYX = 0, rrateXY = 0;
								if (Math.abs(dRealRateX) > 0.01) rrateYX = dRealRateY/dRealRateX;
								else rrateYX = 100000;
								if (Math.abs(dRealRateY) > 0.01) rrateXY = dRealRateX/dRealRateY;
								else rrateXY = 100000;
								double ddX = (dOdomSobra/Math.sqrt(1 + rrateYX*rrateYX))*(iii+1.0)/(1.0*countCount), 
									   ddY = (dOdomSobra/Math.sqrt(1 + rrateXY*rrateXY))*(iii+1.0)/(1.0*countCount);
								if (Math.abs(dRealRateX) > 0.01) ddX = (dRealRateX/Math.abs(dRealRateX))*ddX;
								if (Math.abs(dRealRateY) > 0.01) ddY = (dRealRateY/Math.abs(dRealRateY))*ddY;
								log.debug("(antes)     posesReal["+(iOdom - countCount + iii)+"].getX(): " + posesReal[(iOdom - countCount + iii)].getX() + ", posesReal["+(iOdom - countCount + iii)+"].getY(): " + posesReal[(iOdom - countCount + iii)].getY());
								posesReal[iOdom - countCount + iii] = new Pose2D(posesReal[iOdom - countCount + iii].getX() + ddX, 
															posesReal[iOdom - countCount + iii].getY() + ddY, 
															posesReal[iOdom - countCount + iii].getTheta());
								log.debug("(corrigido) posesReal["+(iOdom - countCount + iii)+"].getX(): " + posesReal[(iOdom - countCount + iii)].getX() + ", posesReal["+(iOdom - countCount + iii)+"].getY(): " + posesReal[(iOdom - countCount + iii)].getY());
								log.debug("iii: "+iii+", ddX: "+ddX+", countCount: " + countCount + ", dRealRateX: "+dRealRateX+", rrateYX: " + rrateYX);
							}
							dOdomSobra = 0; countCount = 0;

							posesReal[idxMatch] = new Pose2D(arrayPoseReal[idxLine].getX(), arrayPoseReal[idxLine].getY(), 0);
							for (; iOdom <= idxMatch; iOdom++)
							{
								posesReal[iOdom] = posesReal[idxMatch];
								log.debug("estouro: posesReal["+(iOdom)+"].getX(): " + posesReal[(iOdom)].getX() + ", posesReal["+(iOdom)+"].getY(): " + posesReal[(iOdom)].getY());
							}

							double dx = posesReal[idxMatch].getX() - posesReal[idxMatch - 1].getX(),
								   dy = posesReal[idxMatch].getY() - posesReal[idxMatch - 1].getY();
							posesReal[idxMatch].setTheta(FunctionsR.angle(dx, dy));
							log.debug("("+idxMatch+"): posesReal["+(idxMatch)+"].getX(): " + posesReal[(idxMatch)].getX() + ", posesReal["+(idxMatch)+"].getY(): " + posesReal[(idxMatch)].getY());

							idxLast = idxLine;
							idxLastOdom = idxMatch;
						}
					}
				}

				log.debug("idxTime: " + idxTime);
				idxTime = idxLastOdom; double dSobra = 0;
				for (int idxLine = idxLineFirst + 1; idxLine < listLinesReal.size(); idxLine++)
				{
					String strLine = (String) listLinesReal.get(idxLine);
					StringTokenizer strToks = new StringTokenizer(strLine, ",:"); String strPre = strToks.nextToken();
					if (!strPre.equals("r"))					
						{log.error("dado de sensor não encontrado: " + strLine); continue;}

					int x = Integer.parseInt(strToks.nextToken()), y = -Integer.parseInt(strToks.nextToken());
					int ang = Integer.parseInt(strToks.nextToken());
					long timemillis = Long.parseLong(strToks.nextToken());
					int idxMatch = Integer.parseInt(strToks.nextToken());
					Pose2D pose = new Pose2D(x, y, 0);

					log.debug(idxTime + ":x: " + x + ", y: " + y + ", idxMatch: " + idxMatch);
					if (idxTime == 0)
					{
						poseIni = pose;
						realIdx[idxTime] = (idxLine+1);
						posesReal[idxTime++] = pose.sub(poseIni);
						log.debug("posesReal["+(idxTime-1)+"].getX(): " + posesReal[(idxTime-1)].getX() + ", posesReal["+(idxTime-1)+"].getY(): " + posesReal[(idxTime-1)].getY());
					}
					else
					{
						pose = pose.sub(poseIni);
						Pose2D poseLast = posesReal[idxTime - 1];
						double dReal = poseLast.getDistance(pose)*0.999975 - dSobra, dReal2 = dReal*dReal;

						log.debug("poseLast.getX(): " + poseLast.getX() + ", poseLast.getY(): " + poseLast.getY());
						log.debug("pose.getX(): " + pose.getX() + ", pose.getY(): " + pose.getY());
						log.debug("posesOdom[idxTime-1].getX(): " + posesOdom[idxTime-1].getX() + ", posesOdom[idxTime-1].getY(): " + posesOdom[idxTime-1].getY());
						log.debug("posesOdom[idxTime].getX(): " + posesOdom[idxTime].getX() + ", posesOdom[idxTime].getY(): " + posesOdom[idxTime].getY());

						int idxTmp = idxTime - 1;
						double dOdomAcc = distances[idxTime], dOdomAcc2 = dOdomAcc*dOdomAcc; 
						while (idxTmp < timeSize && dOdomAcc2 < dReal2)
						{
							++idxTmp;
							double dxP = posesOdom[idxTmp].getX() - posesOdom[idxTime - 1].getX(), 
								   dyP = posesOdom[idxTmp].getY() - posesOdom[idxTime - 1].getY();
							dOdomAcc2 = dxP*dxP + dyP*dyP;
						}
						dOdomAcc = Math.sqrt(dOdomAcc2); 
						dSobra = dOdomAcc - dReal;
						log.debug(idxTmp + ":dOdomAcc: " + dOdomAcc + ", dReal: " + dReal + ", dSobra: " + dSobra);

						posesReal[idxTmp] = pose;
						realIdx[idxTmp] = (idxLine+1);

						double dxT = posesReal[idxTmp].getX() - posesReal[idxTime - 1].getX(), 
							   dyT = posesReal[idxTmp].getY() - posesReal[idxTime - 1].getY();
						double dxOdomT = posesOdom[idxTmp].getX() - posesOdom[idxTime - 1].getX(), 
							   dyOdomT = posesOdom[idxTmp].getY() - posesOdom[idxTime - 1].getY();
						double dTot = Math.sqrt(dxOdomT*dxOdomT + dyOdomT*dyOdomT);
						for (int idx = idxTime; idx < idxTmp; idx++)
						{
							double dxP = posesOdom[idx].getX() - posesOdom[idxTime - 1].getX(), 
								   dyP = posesOdom[idx].getY() - posesOdom[idxTime - 1].getY();
							double dAcc = Math.sqrt(dxP*dxP + dyP*dyP);
							posesReal[idx] = new Pose2D(poseLast.getX() + dxT*dAcc/dTot, poseLast.getY() + dyT*dAcc/dTot, 0);

							double dx = posesReal[idx].getX() - posesReal[idx-1].getX(),
								   dy = posesReal[idx].getY() - posesReal[idx-1].getY();
							posesReal[idx].setTheta(FunctionsR.angle(dx, dy));

							log.debug("posesReal["+idx+"].getX(): " + posesReal[idx].getX() + ", posesReal["+idx+"].getY(): " + posesReal[idx].getY());
							realIdx[idx] = realIdx[idxTime - 1] + ((idx - idxTime)*1.0)/(1.0*(idxTmp - idxTime));
						}
						{
							double dx = posesReal[idxTmp].getX() - posesReal[idxTmp-1].getX(),
								   dy = posesReal[idxTmp].getY() - posesReal[idxTmp-1].getY();
							posesReal[idxTmp].setTheta(FunctionsR.angle(dx, dy));
						}
						idxTime = idxTmp;
						log.debug("posesReal["+idxTime+"].getX(): " + posesReal[idxTime].getX() + ", posesReal["+idxTime+"].getY(): " + posesReal[idxTime].getY());
						idxTime++;
					}
				}
				log.debug("real:idxTime: " + idxTime);
				if (idxTime < timeSize)
					timeSize = idxTime;
			}

		}
		catch (NumberFormatException e)
		{
			log.error("erro", e);
		}
		reader.close();
	}


	protected AbstractDoubleVector getActualRealPose()
	{
		if (poseActual != null)
			return poseActual.convert2vector(); 
		else
			return null;
	}


	protected Pose2D poseActual = null;
	protected void gatherData(boolean bEngine)
	{
		poseActual = posesReal[currentStep];
		poseActual = poseActual.add(poseIni);
		log.debug("("+currentStep+") poseActual: " + poseActual);


		if (!bEngine)
		{	
			Iterator itTrakers = listRobotTrack.iterator();
			while (itTrakers.hasNext())
			{
				EstimatorRenderer track = (EstimatorRenderer) itTrakers.next();
				track.newPose(poseActual);
			}
		}

//		log.debug("posesOdom[currentStep]: " + posesOdom[currentStep]);
		if (odoSns != null)
		{
			odoSns.setReadings(posesOdom[currentStep]);

			odomCountProcessed++;
			if (currentStepOdomLast > 0)
				dOdomMean += posesReal[currentStep].getDistance(posesReal[currentStepOdomLast]);
			currentStepOdomLast = currentStep;
		}
		if (sonSns != null)
		{
			sonSns.setReadings(sonarReadings[currentStep]);

			for (int j = 0; j < 8; j++)
			{
				if (sonarReadings[currentStep][j] != -1)
				{
					sonCountProcessed[j]++;
					if (currentStepSonLast[j] > 0)
						dSonMean[j] += posesReal[currentStep].getDistance(posesReal[currentStepSonLast[j]]);
					currentStepSonLast[j] = currentStep;
				}
			}
		}
		if (vlineMapSns != null || vlineStateSns != null)
		{
			if (vlineReadings[currentStep] != null)
			{
				BufferedImage buffImg = ImageUtil.getImageBMP(vlineReadings[currentStep]);
				int [] imgData = ImageUtil.getThreeBandPackedData(buffImg);
				int imgWidth = buffImg.getWidth();
				int imgHeight = buffImg.getHeight();

				if (vlineMapSns != null)
					vlineMapSns.setReadings(imgData, imgWidth, imgHeight);
				if (vlineStateSns != null)
					vlineStateSns.setReadings(imgData, imgWidth, imgHeight);

				imgCountProcessed++;
				if (currentStepImgLast > 0)
					dImgMean += posesReal[currentStep].getDistance(posesReal[currentStepImgLast]);
				currentStepImgLast = currentStep;
			}
			else
			{
				if (vlineMapSns != null)
					vlineMapSns.setReadings(null, 1, 1);
				if (vlineStateSns != null)
					vlineStateSns.setReadings(null, 1, 1);
			}
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


	public void logStats()
	{
		int meanSonCount = 0;
		double meanSonDist = 0;
		for (int i = 0; i < 8; i++)
		{
			meanSonCount += sonCountProcessed[i];
			meanSonDist += dSonMean[i];
		}
		meanSonCount /= 8;
		meanSonDist /= 8;

		logStats.debug("STATS SONAR:");
		if (meanSonCount > 1)
		{
			logStats.debug("\tnúmero de leituras médio por sonar: " + meanSonCount);
			logStats.debug("\tdistância média percorrida entre leituras: " + meanSonDist/(meanSonCount-1));
		}

		logStats.debug("STATS ODÔMETRO:");
		if (odomCountProcessed > 0)
		{
			logStats.debug("\tnúmero de leituras: " + odomCountProcessed);
			logStats.debug("\tdistância média percorrida entre leituras: " + dOdomMean/(odomCountProcessed-1));
		}

		logStats.debug("STATS VISÃO:");
		if (imgCountProcessed > 1)
		{
			logStats.debug("\tnúmero de leituras: " + imgCountProcessed);
			logStats.debug("\tdistância média percorrida entre leituras: " + dImgMean/(imgCountProcessed-1));
		}
	}


	protected void logData(boolean bEngine)
	{
		if (!bEngine)
		{
			String strInfo = "it: " + currentStep + ", odom: " + odomIdx[currentStep] + ", real: " + nFrmt.format(realIdx[currentStep]);
			this.getInfoSink().setInfo(1, strInfo);

		}
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
		else if (sns instanceof VLineMapSensor)
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
}

