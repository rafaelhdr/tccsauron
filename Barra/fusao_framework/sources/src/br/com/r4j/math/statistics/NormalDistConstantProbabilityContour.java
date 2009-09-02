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


/** @modelguid {FB946FBB-A10D-4C2E-8BB8-1B2D09C95520} */
public class NormalDistConstantProbabilityContour implements ConstantProbabilityContour
{
	/** @modelguid {71EABB5A-E97F-4DE9-ACC4-EBF0EA78FBCC} */
	private static Log log = LogFactory.getLog(NormalDistConstantProbabilityContour.class.getName());

	/** @modelguid {CC122348-AE69-43BB-97BC-99E89E512140} */
	private AbstractDoubleSquareMatrix covar = null; 
	/** @modelguid {5CA14146-E500-4764-90AB-2CF75CE08EFD} */
	private AbstractDoubleVector mean = null; 

	/** @modelguid {DBC08734-CB81-46FC-B36D-86BF39281B7B} */
	private AbstractDoubleVector [] eigenvectors = null;
	/** @modelguid {143036CF-43AF-4712-A208-5BC0B0EFF47E} */
	private double [] arrayEigenValues = null;

	/** @modelguid {0A6B8294-EF05-4663-815C-C7EF75DDF1D5} */
	private double c = 0;


	/** @modelguid {483B664F-8433-4301-8917-587BFF011ABF} */
	public NormalDistConstantProbabilityContour()
	{
		c = 1;
	}


	/** @modelguid {1282B597-EF00-4013-B14E-F6C4C11C17CF} */
	public void setAccumulatedProbability(double accProb)
	{
		ChiSqrDistribution dist = new ChiSqrDistribution(2);
		c = dist.inverse(accProb);
		log.debug("dist.inverse(accProb) = " + c + ", dist.inverse(1- accProb) = " + dist.inverse(1- accProb));
	}


	/** @modelguid {818DE712-3FCE-4427-B831-A0A8B1686B76} */
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


	/** @modelguid {A39F86B5-8C93-4A3D-915E-7F64B850D0AE} */
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

