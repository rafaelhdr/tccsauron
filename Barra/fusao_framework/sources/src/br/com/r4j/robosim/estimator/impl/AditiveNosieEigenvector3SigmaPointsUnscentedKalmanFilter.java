package br.com.r4j.robosim.estimator.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.estimator.EstimationException;
import br.com.r4j.robosim.estimator.Sensor;


public class AditiveNosieEigenvector3SigmaPointsUnscentedKalmanFilter extends AditiveNosieUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieEigenvector3SigmaPointsUnscentedKalmanFilter.class.getName());
	private static Log logEigenUKF = LogFactory.getLog("eigenukf");

	
	public AditiveNosieEigenvector3SigmaPointsUnscentedKalmanFilter()
	{
		super();
		this.logEstimator = logEigenUKF;
	}


	public String getName()
	{
		return "Filtro de Kalman Unscented Eigenvector";
	}


	protected AbstractDoubleMatrix generateSigmaPoints(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		AbstractDoubleMatrix [] arraySqrts = null;

		arraySqrts = JSciMatrixMath.choleskyDecompose(covar);
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("arraySqrts[0]: \r\n" + MatrixUtil.toString(arraySqrts[0], 7, 6));
			logEstimator.debug("covar busted: \r\n" + MatrixUtil.toString(covar, 9, 4));
		}
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
		}
		
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));
		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			{
				double val = gammaSqrt*arraySqrts[0].getElement(idxVar, idxObj);
				if (idxObj < 3)
					val = 0;
				sigma.setElement(idxVar, idxObj + 1, mean.getComponent(idxVar) + val);
				sigma.setElement(idxVar, idxObj + 1 + mean.dimension(), mean.getComponent(idxVar) - val);
			}
		}

		double sigXd2 = gammaSqrt*arraySqrts[0].getElement(0, 0)/1.2;
		double sigYd2 = gammaSqrt*arraySqrts[0].getElement(1, 1)/1.2;
		double sigTheta = gammaSqrt*arraySqrts[0].getElement(2, 2); 

		sigma.setElement(0, 0 + 1, mean.getComponent(0) + sigXd2);
		sigma.setElement(0, 1 + 1, mean.getComponent(0) - sigXd2);
		sigma.setElement(0, 0 + 1 + mean.dimension(), mean.getComponent(0) + sigXd2);
		sigma.setElement(0, 1 + 1 + mean.dimension(), mean.getComponent(0) - sigXd2);
		sigma.setElement(1, 0 + 1, mean.getComponent(1) + sigYd2);
		sigma.setElement(1, 1 + 1, mean.getComponent(1) + sigYd2);
		sigma.setElement(1, 0 + 1 + mean.dimension(), mean.getComponent(1) - sigYd2);
		sigma.setElement(1, 1 + 1 + mean.dimension(), mean.getComponent(1) - sigYd2);

		sigma.setElement(2, 2 + 1, mean.getComponent(2) + sigTheta);
		sigma.setElement(2, 2 + 1 + mean.dimension(), mean.getComponent(2) - sigTheta);
		
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigma: \r\n" + MatrixUtil.toString(sigma, 8, 5));
		return sigma;
	}

	
	public static ModelFilter getModelFilter()
	{
		return new Eigen3UKFModelFilter();
	}
}


class Eigen3UKFModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return (dynModel instanceof UKFDoubleVectorFunction);
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return (sensModel instanceof UKFDoubleVectorFunction);
	}
}

