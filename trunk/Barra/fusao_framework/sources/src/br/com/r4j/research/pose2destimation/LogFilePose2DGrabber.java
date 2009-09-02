package br.com.r4j.research.pose2destimation;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.jmr.datasource.RobotDescription;
import br.com.r4j.robosim.Pose2D;


/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 */
public class LogFilePose2DGrabber implements Pose2DGrabber
{
	private static Log log = LogFactory.getLog(LogFilePose2DGrabber.class.getName());

	private RobotDescription robotDesc = null;
	private long [] arrayTimes = null;
	private List [] arraySensorMeasures = null;
	private Pose2D [] arrayRobotPosition = null;
	private Pose2D [] arrayRobotMove = null;
	private AbstractDoubleSquareMatrix [] arrayRobotMoveCovar = null;
	private Pose2D poseOffset = null;
	private Pose2D poseRobotIni = null;

	private String strDataFile = null;
	private int actIdx = -1;
	
	private int subIdx = 0;
	
	
	public LogFilePose2DGrabber()
	{
		this.actIdx = -1;
	}

	public void setInitalState(Pose2D poseRobotIni)	{this.poseRobotIni = poseRobotIni;}
	public void setRobotDescription(RobotDescription robotDesc)	{this.robotDesc = robotDesc;}
	

	/**
	 * Retorna o pose que está no arquivo. Não deve se usado a não ser para debug.
	 *
	 */
	public Pose2D getPose(long imageTimestamp)
	{
		return arrayRobotPosition[actIdx];
	}


	/**
	 * Retorna o movimento do robô considerando a última posição como sendo
	 * a origem do eixo de coordenadas.
	 *
	 * Para se usar a informação, deve-se converter os valores de X e Y para o sistema
	 * de coordenadas em qual o robô está isnerido.
	 *
	 */
	public Pose2D getMovement(long imageTimestamp)
	{
		return arrayRobotMove[actIdx];
/*
		AbstractDoubleMatrix covarTotal = new DoubleMatrix(3, 3);
		AbstractDoubleMatrix covarTotalOld = covarTotal;
		if (actIdx > 0)
			covarTotalOld = arrayRobotPosition[actIdx - 1 - subIdx].getCovar();
		AbstractDoubleMatrix covarTotalActual = arrayRobotPosition[actIdx].getCovar();
		covarTotal.setElement(0, 0, covarTotalActual.getElement(0, 0) - covarTotalOld.getElement(0, 0));
		covarTotal.setElement(1, 1, covarTotalActual.getElement(1, 1) - covarTotalOld.getElement(1, 1));
		covarTotal.setElement(2, 2, covarTotalActual.getElement(2, 2) - covarTotalOld.getElement(2, 2));
		if (actIdx > 0)
			return new Pose2D(arrayRobotPosition[actIdx].getX() - arrayRobotPosition[actIdx-1 - subIdx].getX(), arrayRobotPosition[actIdx].getY() - arrayRobotPosition[actIdx-1 - subIdx].getY(), arrayRobotPosition[actIdx].getTheta() - arrayRobotPosition[actIdx-1 - subIdx].getTheta(), covarTotal);
		else
			return new Pose2D(0, 0, 0, covarTotal);
//*/
	}


	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp)
	{
		return null;
	}


	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp)
	{
		return arrayRobotMoveCovar[actIdx];
	}


	public void accumulateMovement()
	{
		subIdx++;
	}


	public void resetMovement()
	{
		subIdx = 0;
	}


	public void setDataFile(String strDataFile)
	{
		this.strDataFile = strDataFile;
	}


	public void setPoseOffset(Pose2D poseOffset)
	{
		this.poseOffset = poseOffset;
	}


	public int getNumberOfTimestamps()
	{
		return arrayRobotPosition.length;
	}


 	public boolean hasNextTimestamp()
	{
		return actIdx < arrayRobotPosition.length - 1;
	}


	public long getNextTimestamp()
	{
		actIdx++;
		return arrayTimes[actIdx];
	}


	public void initFileParser()
	{
		this.actIdx = -1;
		if (poseOffset == null)
			poseOffset = new Pose2D(0, 0, 0);

		ArrayList listData = new ArrayList();
		ArrayList listIsDataPos = new ArrayList();
		char [] arrayDataBuffer = new char[2000];
		try
		{
			String strLine = null;
			FileReader fRead = new FileReader(strDataFile);
			strLine = this.getLine(fRead, arrayDataBuffer);
			int countRobotData = 0, countSonarData = 0;
			while (strLine != null)
			{
				if (strLine.startsWith("s"))
				{
					listData.add(strLine.substring(2));
					listIsDataPos.add(Boolean.FALSE);
					countSonarData++;
				}
				else
				{
					listData.add(strLine.substring(3));
					listIsDataPos.add(Boolean.TRUE);
					countRobotData++;
				}
				strLine = this.getLine(fRead, arrayDataBuffer);
			}
			arrayTimes = new long[countRobotData];
			arraySensorMeasures = new List[countRobotData];
			arrayRobotPosition = new Pose2D[countRobotData];
			arrayRobotMove = new Pose2D[countRobotData];
			arrayRobotMoveCovar = new AbstractDoubleSquareMatrix[countRobotData];

			HashMap mapTime2Idx = new HashMap();
			int idx = 0;
			long maxTime = 0, timeOffset = 0;
			Pose2D posRobotOld = poseRobotIni;
			Iterator itData = listData.iterator();
			Iterator itIsDataPos = listIsDataPos.iterator();
			while (itData.hasNext())
			{
				String strData = (String) itData.next();
				Boolean bIsRobot = (Boolean) itIsDataPos.next();
				if (Boolean.TRUE.equals(bIsRobot))
				{
					String strRobotData = strData;
					String strXRobo = strRobotData.substring(0, strRobotData.indexOf(","));
					int xRobo = Integer.parseInt(strXRobo);
					strRobotData = strRobotData.substring(strRobotData.indexOf(",") + 1);
					String strYRobo = strRobotData.substring(0, strRobotData.indexOf(","));

					// MUDANÇA NO CÁLCULO DO POSE!!!
					int yRobo = Integer.parseInt(strYRobo);
					strRobotData = strRobotData.substring(strRobotData.indexOf(",") + 1);
					String strThetaRobo = strRobotData.substring(0, strRobotData.indexOf(")"));
					int thetaRobo = Integer.parseInt(strThetaRobo);
					strRobotData = strRobotData.substring(strRobotData.indexOf(")") + 5);
					long dtOri = Long.parseLong(strRobotData);
					long dt = dtOri + timeOffset;
					if (dt > maxTime)
					{
						maxTime = dt;
					}
					else if (dt < maxTime)
					{
						timeOffset = (maxTime + 1) - dtOri;
						dt = maxTime + 1;
					}

					Pose2D posRobotMove = null;
					Pose2D posRobot = new Pose2D(xRobo + poseOffset.getX(), yRobo + poseOffset.getY(), thetaRobo * Math.PI / 180 + poseOffset.getTheta());
					DoubleSquareMatrix stateCov = new DoubleSquareMatrix(3);
					if (posRobotOld == null)
					{
						posRobotMove = new Pose2D(0, 0, 0);
					}
					else
					{
						double dX = posRobot.getX() - posRobotOld.getX();
						double dY = posRobot.getY() - posRobotOld.getY();
						double dMove = Math.sqrt(dX*dX + dY*dY);
						double thetaLin = 0;
						if (dX > 0)
						{
							double dDiv = Math.atan(Math.abs(dY/dX));
							if (dY > 0)
								thetaLin = dDiv;
							else
								thetaLin = -dDiv;
						}
						else if (dX < 0)
						{
							double dDiv = Math.atan(Math.abs(dY/dX));
							if (dY > 0)
								thetaLin = Math.PI - dDiv;
							else
								thetaLin = -Math.PI + dDiv;
						}
						thetaLin -= posRobotOld.getTheta();
						double thetaLinSin = Math.sin(thetaLin), thetaLinCos = Math.cos(thetaLin);
						double dxLin = thetaLinCos*dMove;
						double dyLin = thetaLinSin*dMove;
						double dTheta = posRobot.getTheta() - posRobotOld.getTheta();
						posRobotMove = new Pose2D(dxLin, dyLin, dTheta);
						stateCov.setElement(0, 0, robotDesc.getDXVariancePerMilimiter()*dX*dX);
						stateCov.setElement(1, 1, robotDesc.getDYVariancePerMilimiter()*dY*dY);
						if (Math.abs(dTheta) < 5*Math.PI/180)
							stateCov.setElement(2, 2, robotDesc.getDThetaVariancePerMillimiter()*5*Math.PI/180*5*Math.PI/180);
						else
							stateCov.setElement(2, 2, robotDesc.getDThetaVariancePerMillimiter()*dTheta*dTheta);
					}
					posRobotOld = posRobot;
					arrayTimes[idx] = dt;
					arrayRobotPosition[idx] = posRobot;
					arrayRobotMove[idx] = posRobotMove;
					arrayRobotMoveCovar[idx] = stateCov;

					mapTime2Idx.put(new Long(dt), new Integer(idx));
					idx++;
				}
				else
				{
					String strSonarData = strData;
					String strNumSon = strSonarData.substring(0, strSonarData.indexOf(":"));
					int sonarNum = Integer.parseInt(strNumSon);
					strSonarData = strSonarData.substring(strSonarData.indexOf(":") + 3);
					String strRange = strSonarData.substring(0, strSonarData.indexOf(":"));
					int range = Integer.parseInt(strRange);
					strSonarData = strSonarData.substring(strSonarData.indexOf(":") + 2);
					String strXRobo = strSonarData.substring(0, strSonarData.indexOf(","));
					int xRobo = Integer.parseInt(strXRobo);
					strSonarData = strSonarData.substring(strSonarData.indexOf(",") + 1);
					String strYRobo = strSonarData.substring(0, strSonarData.indexOf(","));
					int yRobo = Integer.parseInt(strYRobo);
					strSonarData = strSonarData.substring(strSonarData.indexOf(",") + 1);
					String strThetaRobo = strSonarData.substring(0, strSonarData.indexOf(")"));
					int thetaRobo = Integer.parseInt(strThetaRobo);
					strSonarData = strSonarData.substring(strSonarData.indexOf(")") + 5);
					long dtOri = Long.parseLong(strSonarData);
					long dt = dtOri + timeOffset;

					AbstractDoubleMatrix covarInc = new DoubleMatrix(3, 3);
					covarInc.setElement(0, 0, xRobo*robotDesc.getDXVariancePerMilimiter());
					covarInc.setElement(1, 1, yRobo*robotDesc.getDYVariancePerMilimiter());
					covarInc.setElement(2, 2, thetaRobo * Math.PI / 180*robotDesc.getDThetaVariancePerRad());
					Pose2D posRobot = new Pose2D(xRobo + poseOffset.getX(), yRobo + poseOffset.getY(), thetaRobo * Math.PI / 180 + poseOffset.getTheta());
				}
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			log.error("Não foi possível caregar o mapa!");
			log.error(e);
			return;
		}

	}

	private int nextReaded = -1;
	private boolean isNextAlreadyReaded = false;
	private boolean reachedNewLine = false;
	
	private String getLine(FileReader fRead, char [] arrayDataBuffer) throws IOException
	{
		int counter = 0;
		if (!isNextAlreadyReaded)
			nextReaded = (int) fRead.read();
		else
			isNextAlreadyReaded = false;

		while (nextReaded != '\n' && nextReaded != '\r')
		{
			if (nextReaded == -1)
				return null;
			arrayDataBuffer[counter++] = (char) nextReaded;
			nextReaded = (int) fRead.read();
		}
		while ((nextReaded == '\r' || nextReaded == '\n') && nextReaded != -1)
		{
			nextReaded = (int) fRead.read();
		}
		reachedNewLine = true;
		isNextAlreadyReaded = true;
		return new String(arrayDataBuffer, 0, counter);
	}
}

