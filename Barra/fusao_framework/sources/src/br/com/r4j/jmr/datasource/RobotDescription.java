package br.com.r4j.jmr.datasource;

import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.Collections;
import br.com.r4j.commons.util.IntegerUtil;
import br.com.r4j.commons.util.LabelValuePair;


/**
 */
public class RobotDescription
{
	private static Log log = LogFactory.getLog(RobotDescription.class.getName());

	public int xS0 = 115;
	public int xS1 = 155;
	public int xS2 = 190;
	public int xS3 = 210;
	public int xS4 = 210;
	public int xS5 = 190;
	public int xS6 = 155;
	public int xS7 = 115;
	public int yS0 = 130;
	public int yS1 = 115;
	public int yS2 = 80;
	public int yS3 = 25;
	public int yS4 = -25;
	public int yS5 = -80;
	public int yS6 = -115;
	public int yS7 = -130;
	public double angS0 = 90 * Math.PI / 180;
	public double angS1 = 50 * Math.PI / 180;
	public double angS2 = 30 * Math.PI / 180;
	public double angS3 = 10 * Math.PI / 180;
	public double angS4 = -10 * Math.PI / 180;
	public double angS5 = -30 * Math.PI / 180;
	public double angS6 = -50 * Math.PI / 180;
	public double angS7 = -90 * Math.PI / 180;

	public int [] xS = null;
	public int [] yS = null;
	public double [] angS = null;
	public double [] dS = null;

	public float sonarMaxReading = 2500;

	public int sonarNum = 8;

	public double sonarHalfVisibilityAngle = 15 * Math.PI / 180;

	public int robotPositionXIdx = sonarNum;
	public int robotPositionYIdx = sonarNum + 1;
	public int robotPositionThetaIdx = sonarNum + 2;

	public int sonarExpectedReadingsRangeBeginIdx = sonarNum + 3;
	public int sonarExpectedReadingsAngleBeginIdx = sonarNum + 3 + sonarNum;

	public int varsNum = sonarExpectedReadingsAngleBeginIdx + sonarNum;

	public List listSonarLabels = Collections.createList(
								new LabelValuePair("s0", IntegerUtil.ZERO),
								new LabelValuePair("s1", IntegerUtil.ONE),
								new LabelValuePair("s2", IntegerUtil.TWO),
								new LabelValuePair("s3", IntegerUtil.THREE),
								new LabelValuePair("s4", IntegerUtil.FOUR),
								new LabelValuePair("s5", IntegerUtil.FIVE),
								new LabelValuePair("s6", IntegerUtil.SIX),
								new LabelValuePair("s7", IntegerUtil.SEVEN),
								new LabelValuePair("robo X", new Integer(robotPositionXIdx)),
								new LabelValuePair("robo Y", new Integer(robotPositionYIdx)),
								new LabelValuePair("robo Theta", new Integer(robotPositionThetaIdx)),
								new LabelValuePair("s0 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 0)),
								new LabelValuePair("s1 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 1)),
								new LabelValuePair("s2 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 2)),
								new LabelValuePair("s3 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 3)),
								new LabelValuePair("s4 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 4)),
								new LabelValuePair("s5 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 5)),
								new LabelValuePair("s6 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 6)),
								new LabelValuePair("s7 exp", new Integer(sonarExpectedReadingsRangeBeginIdx + 7)),
								new LabelValuePair("s0 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 0)),
								new LabelValuePair("s1 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx+ 1)),
								new LabelValuePair("s2 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 2)),
								new LabelValuePair("s3 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 3)),
								new LabelValuePair("s4 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 4)),
								new LabelValuePair("s5 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 5)),
								new LabelValuePair("s6 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 6)),
								new LabelValuePair("s7 angle exp", new Integer(sonarExpectedReadingsAngleBeginIdx + 7))
							);


	public RobotDescription()
	{
		xS = new int[8];
		yS = new int[8];
		angS = new double[8];
		dS = new double[8];

		int i = 0;
		xS[i] = xS0; i++;
		xS[i] = xS1; i++;
		xS[i] = xS2; i++;
		xS[i] = xS3; i++;
		xS[i] = xS4; i++;
		xS[i] = xS5; i++;
		xS[i] = xS6; i++;
		xS[i] = xS7; i++;

		i = 0;
		yS[i] = yS0; i++;
		yS[i] = yS1; i++;
		yS[i] = yS2; i++;
		yS[i] = yS3; i++;
		yS[i] = yS4; i++;
		yS[i] = yS5; i++;
		yS[i] = yS6; i++;
		yS[i] = yS7; i++;

		i = 0;
		angS[i] = angS0; i++;
		angS[i] = angS1; i++;
		angS[i] = angS2; i++;
		angS[i] = angS3; i++;
		angS[i] = angS4; i++;
		angS[i] = angS5; i++;
		angS[i] = angS6; i++;
		angS[i] = angS7; i++;

		i = 0;
		dS[i] = Math.sqrt(xS0*xS0 + yS0*yS0); i++;
		dS[i] = Math.sqrt(xS1*xS1 + yS1*yS1); i++;
		dS[i] = Math.sqrt(xS2*xS2 + yS2*yS2); i++;
		dS[i] = Math.sqrt(xS3*xS3 + yS3*yS3); i++;
		dS[i] = Math.sqrt(xS4*xS4 + yS4*yS4); i++;
		dS[i] = Math.sqrt(xS5*xS5 + yS5*yS5); i++;
		dS[i] = Math.sqrt(xS6*xS6 + yS6*yS6); i++;
		dS[i] = Math.sqrt(xS7*xS7 + yS7*yS7); i++;
	}


	public boolean isSonarIndex(int idx)
	{
		return idx > -1 && idx < 8;
	}


	public double getSonarReadingVariance(double range, double angle)
	{
		return 0.1*range;
	}


	public double getSonarReadingVariance(double range)
	{
		return 0.1*range;
	}


	public double getDXVariancePerMilimiter()
	{
//		return 16*0.001;
		return 100*0.001;
//		return 1;
	}


	public double getDYVariancePerMilimiter()
	{
//		return 16*0.001;
		return 100*0.001;
//		return 1;
	}


	public double getDThetaVariancePerRad()
	{
		return 4*0.05;
//		return 1;
	}


	public double getDThetaVariancePerMillimiter()
	{
		return 4*0.025/1000/1000;
//		return 1;
	}
}

