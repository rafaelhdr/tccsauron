package br.com.r4j.math.statistics;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.LinearMath;
import JSci.maths.MaximumIterationsExceededException;
import JSci.maths.statistics.ChiSqrDistribution;


public class NormalDistConstantProbabilityContour implements ConstantProbabilityContour
{
	private static Log log = LogFactory.getLog(NormalDistConstantProbabilityContour.class.getName());

	private AbstractDoubleSquareMatrix covar = null; 
	private AbstractDoubleVector mean = null; 

	private AbstractDoubleVector [] eigenvectors = null;
	private double [] arrayEigenValues = null;

	private double c = 0;


	public NormalDistConstantProbabilityContour()
	{
		c = 1;
	}


	public void setAccumulatedProbability(double accProb)
	{
		ChiSqrDistribution dist = new ChiSqrDistribution(2);
		c = dist.inverse(accProb);
		log.debug("dist.inverse(accProb) = " + c + ", dist.inverse(1- accProb) = " + dist.inverse(1- accProb));
	}


	public void setNormalDistribuition(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar)
	{
//		log.debug("setNormalDistribuition = " + mean);
//		log.debug("setNormalDistribuition = " + covar);
		try
		{
			this.mean = mean;
			this.covar = covar;
			this.eigenvectors = new AbstractDoubleVector[2];
			this.arrayEigenValues = LinearMath.eigenSolveSymmetric(covar, eigenvectors);
//			log.debug("eigenvectors = " + eigenvectors);
//			log.debug("eigenvectors[0] = " + eigenvectors[0]);
//			log.debug("eigenvectors[1] = " + eigenvectors[1]);
		}
		catch (MaximumIterationsExceededException e)
		{
			log.error(e);
		}

	}


	public Shape getContourAsShape()
	{
		double A = Math.sqrt(arrayEigenValues[0])*c;
		double B = Math.sqrt(arrayEigenValues[1])*c;
		double theta1 = Math.atan2(eigenvectors[0].getComponent(1), eigenvectors[0].getComponent(0));

		Ellipse2D.Double ellipse = new Ellipse2D.Double(-A, -B, 2*A, 2*B);
		AffineTransform trafoRotAndMove = new AffineTransform();
		trafoRotAndMove.rotate(theta1);
		trafoRotAndMove.translate(mean.getComponent(0), mean.getComponent(1));

		return trafoRotAndMove.createTransformedShape(ellipse);
	}
}

