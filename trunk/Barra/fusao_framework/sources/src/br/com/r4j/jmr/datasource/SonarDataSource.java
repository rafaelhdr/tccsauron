package br.com.r4j.jmr.datasource;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import javaGL.Position2D;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import robots.SonarReading;
import br.com.r4j.jmr.SonarReadingHolder;


/**
 * Aceita como entrada um arquivo com entradas da posição do robô e das leituras
 * dos sonares no seguinte formato:
 *		r:(2500,1,2):dt:3			-> posição do robô, x = 2500, y = 1, theta = 2, tempo = 3
 * 		s:1:r:2500:(2501,1,2):dt:3	-> leitura do sonar, sonar 1 (idx), range = 2500, posição do robô na leitura, x = 2501, y = 1, theta = 2, tempo = 3
 *
 *
 * O tempo sempre é considerado crescente. Se for identificado tempo decrescente, então
 * é somado um offset mínimo ao tempo para qeu ele volte a se tornar crescente.
 */
public class SonarDataSource
{
	private static Log log = LogFactory.getLog(SonarDataSource.class.getName());

	private long [] arrayTimes = null;
	private List [] arraySensorMeasures = null;
	private Position2D [] arrayRobotPosition = null;


	public SonarDataSource(String strDataFile)
	{
/*
		ArrayList listSonarData = new ArrayList();
		ArrayList listRobotData = new ArrayList();
/*/
		ArrayList listData = new ArrayList();
		ArrayList listIsDataPos = new ArrayList();

		char [] arrayDataBuffer = new char[2000];
		String strLine = null;
		try
		{
			FileReader fRead = new FileReader(strDataFile);
			strLine = this.getLine(fRead, arrayDataBuffer);
			int countRobotData = 0;
			int countSonarData = 0;
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
			arrayRobotPosition = new Position2D[countRobotData];

			HashMap mapTime2Idx = new HashMap();
			int idx = 0;
			long maxTime = 0, timeOffset = 0;
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

					Position2D posRobot = new Position2D(xRobo, yRobo, thetaRobo * Math.PI / 180);
					arrayTimes[idx] = dt;
					arrayRobotPosition[idx] = posRobot;
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

					Position2D posRobot = new Position2D(xRobo, yRobo, thetaRobo * Math.PI / 180);
					SonarReading reading = new SonarReading(0, 0, range, posRobot);
					SonarReadingHolder hldr = new SonarReadingHolder(sonarNum, reading);

					int idx2 = ((Integer) mapTime2Idx.get(new Long(dt))).intValue();
					if (arraySensorMeasures[idx2] == null)
						arraySensorMeasures[idx2] = new ArrayList();
					arraySensorMeasures[idx2].add(hldr);
				}

			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			System.err.println("Não foi possível caregar o mapa!");
			return;
		}
	}


	public long [] getTimes()	{return arrayTimes;}
	public List [] getSensorMeasures()	{return arraySensorMeasures;}
	public Position2D [] getRobotPosition()	{return arrayRobotPosition;}


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

