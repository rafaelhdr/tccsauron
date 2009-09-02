package br.com.r4j.math.kalmanfilter.test;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.math.kalmanfilter.DoubleVectorFunction;
import br.com.r4j.robosim.Pose2D;


/** 
 *
 */
public class OneFixedRadarDoubleVectorFunction implements DoubleVectorFunction
{
	private static Log log = LogFactory.getLog(OneFixedRadarDoubleVectorFunction.class.getName());
	private static Log logUKF = LogFactory.getLog("ukf");

	private Random rnd = null;

	private Pose2D poseRadar = null;
	private AbstractDoubleSquareMatrix pRadarMeasure = null;

//	private double d = 0;
//	private double dTheta = 0;


	public OneFixedRadarDoubleVectorFunction()
	{
		rnd = new Random((new Date()).getTime());
	}


	public void setRadarPose(Pose2D poseRadar)	{this.poseRadar = poseRadar;}
	public void setRadarPoseCovar(AbstractDoubleSquareMatrix pRadarMeasure)	{this.pRadarMeasure = pRadarMeasure;}

	public void setStates(double d1, double dTheta1)
	{
//		this.d = d1;
//		this.dTheta = dTheta1;
	}


	public AbstractDoubleVector getTrueMeasure(Pose2D poseRobotLast)
	{
		AbstractDoubleVector measures = new DoubleVector(2);

		double dX = poseRobotLast.getX() - poseRadar.getX();
		double dY = poseRobotLast.getY() - poseRadar.getY();
		measures.setComponent(0, Math.sqrt(dX*dX + dY*dY));
		if (dX != 0)
			measures.setComponent(1, Math.atan(dY/dX));
		else if (dY > 0)
			measures.setComponent(1, Math.PI/2);
		else
			measures.setComponent(1, -Math.PI/2);

		return measures;
	}


	public void calculate(AbstractDoubleMatrix input, AbstractDoubleMatrix output)
	{
/*
		double dVar = pRadarMeasure.getElement(0, 0);
		double thetaVar = pRadarMeasure.getElement(1, 1);
		double thetaError = Math.sqrt(thetaVar)*rnd.nextGaussian();
		double dError = Math.sqrt(dVar)*rnd.nextGaussian();
//*/
		for (int idxInput = 0; idxInput < input.columns(); idxInput++)
		{
			double dX = input.getElement(0, idxInput) - poseRadar.getX();
			double dY = input.getElement(1, idxInput) - poseRadar.getY();
			output.setElement(0, idxInput, Math.sqrt(dX*dX + dY*dY));
			if (dX > 0)
			{
				double dDiv = Math.atan(Math.abs(dY/dX));
				if (dY > 0)
					output.setElement(1, idxInput, dDiv);
				else
					output.setElement(1, idxInput, -dDiv);
			}
			else if (dX < 0)
			{
				double dDiv = Math.atan(Math.abs(dY/dX));
				if (dY > 0)
					output.setElement(1, idxInput, Math.PI - dDiv);
				else
					output.setElement(1, idxInput, -Math.PI + dDiv);
			}
		}
	}
}


