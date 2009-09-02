package br.com.r4j.math.kalmanfilter.test;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.math.kalmanfilter.DoubleVectorFunction;
import br.com.r4j.robosim.Pose2D;


/** 
 *
 *
 * 
 */
public class Pose2DMoveStateDoubleVectorFunction implements DoubleVectorFunction
{
	private static Log log = LogFactory.getLog(Pose2DMoveStateDoubleVectorFunction.class.getName());
	private static Log logUKF = LogFactory.getLog("ukf");

	private Random rnd = null;

	private double d = 0;
	private double dTheta = 0;

	private AbstractDoubleSquareMatrix pRobotMove = null;


	public Pose2DMoveStateDoubleVectorFunction()
	{
		rnd = new Random((new Date()).getTime());
	}


	/**
	 * Calcula um pose para um movimento
	 *
	 */
	public Pose2D getPose(Pose2D pose1, double d1, double dTheta1)
	{
		double dThetaSin = Math.sin(dTheta1), dThetaCos = Math.cos(dTheta1);
		double xRobot = pose1.getX() + dThetaCos*d1;
		double yRobot = pose1.getY() + dThetaSin*d1;
		double thetaRobot = pose1.getTheta() + dTheta1;

		return new Pose2D(xRobot, yRobot, thetaRobot);
	}

		
		
	public void setRobotMoveCovar(AbstractDoubleSquareMatrix pRobotMove)	{this.pRobotMove = pRobotMove;}

	public AbstractDoubleSquareMatrix getStateTransitionErrorInc()
	{
		AbstractDoubleSquareMatrix ret = new DoubleSquareMatrix(2);

		double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);
		double dX = dThetaCos*d, dY = dThetaSin*d;
		ret.setElement(0, 0, dX*pRobotMove.getElement(0, 0)*dX/1000/1000);
		ret.setElement(1, 1, dY*pRobotMove.getElement(1, 1)*dY/1000/1000);
//		ret.setElement(2, 2, pRobotMove.getElement(2, 2));

//		logUKF.debug("pRobotMove.getElement(0, 0)*dX = " + (pRobotMove.getElement(0, 0)*dX) + ", dX = " + dX);
//		logUKF.debug("ret: \r\n" + MatrixUtil.toString(ret, 7, 4));

		return ret;
	}


	public void setStates(double d1, double dTheta1)
	{
		this.d = d1;
		this.dTheta = dTheta1;
		logUKF.debug("d = " + d + ", dTheta = " + dTheta);
	}


	public void calculate(AbstractDoubleMatrix input, AbstractDoubleMatrix output)
	{
//		double thetaVar = pRobotMove.getElement(2, 2);
		double dVar = Math.sqrt(pRobotMove.getElement(0, 0)*pRobotMove.getElement(0, 0) + pRobotMove.getElement(1, 1)*pRobotMove.getElement(1, 1));
//		double dThetaError = dTheta + dTheta*Math.sqrt(thetaVar)*rnd.nextGaussian();
//		double dError = d + d*Math.sqrt(dVar/4)*rnd.nextGaussian()/1000 + d*Math.sqrt(dVar/4)*rnd.nextDouble()/1000 + Math.abs(d*Math.sqrt(dVar/2)*rnd.nextDouble()/1000);
//		double dError = d + Math.abs(d*Math.sqrt(dVar)*rnd.nextGaussian()/1000);
		double dError = d*(1 + Math.sqrt(dVar)*rnd.nextGaussian()/1000);
		if (rnd.nextDouble() > 0.65)
			dTheta += 15*rnd.nextDouble()*Math.PI/180;

//		logUKF.debug("dVar = " + dVar + ", thetaVar = " + thetaVar + ", dThetaError = " + dThetaError + ", dError = " + dError);

		for (int idxInput = 0; idxInput < input.columns(); idxInput++)
		{
			double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);

			double xRobot = input.getElement(0, idxInput) + dThetaCos*dError;
			double yRobot = input.getElement(1, idxInput) + dThetaSin*dError;
//			double thetaRobot = input.getElement(2, idxInput) + dThetaError;

			output.setElement(0, idxInput, xRobot);
			output.setElement(1, idxInput, yRobot);
//			output.setElement(2, idxInput, thetaRobot);
		}
	}
}


