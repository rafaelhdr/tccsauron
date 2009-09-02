package br.com.r4j.jmr;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import robots.SonarReading;


/**
 */
public class SonarReadingHolder
{
	private static Log log = LogFactory.getLog(SonarReadingHolder.class.getName());

	private int sonarNum = 0;
	private SonarReading reading = null;


	public SonarReadingHolder(int sonarNum, SonarReading reading)
	{
		this.sonarNum = sonarNum;
		this.reading = reading;
	}


	public int getSonarNum()
	{
		return sonarNum;
	}

	public SonarReading getSonarReading()
	{
		return reading;
	}


	public String toString()
	{
		return sonarNum + ":" + reading.dRange;
	}
}

