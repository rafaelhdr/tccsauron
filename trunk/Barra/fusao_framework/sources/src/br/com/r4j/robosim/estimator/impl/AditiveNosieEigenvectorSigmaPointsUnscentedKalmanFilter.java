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


public class AditiveNosieEigenvectorSigmaPointsUnscentedKalmanFilter extends AditiveNosieUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieEigenvectorSigmaPointsUnscentedKalmanFilter.class.getName());
	private static Log logEigenUKF = LogFactory.getLog("eigenukf");

	
	public AditiveNosieEigenvectorSigmaPointsUnscentedKalmanFilter()
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
		AbstractDoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		AbstractDoubleSquareMatrix [] usv = covar.singularValueDecompose();

		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("u: \r\n" + MatrixUtil.toString(usv[0], 7, 6));
			logEstimator.debug("s: \r\n" + MatrixUtil.toString(usv[1], 7, 6));
			logEstimator.debug("v: \r\n" + MatrixUtil.toString(usv[2], 7, 6));
		}

		AbstractDoubleVector zMean = ((AbstractDoubleMatrix) usv[0].transpose()).multiply(mean);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("zMean: \r\n" + MatrixUtil.toString(zMean, 9, 4));
		
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, zMean.getComponent(idxVar));

		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			double val = gammaSqrt*Math.sqrt(usv[1].getElement(idxObj, idxObj));
			sigma.setElement(idxObj, idxObj + 1, zMean.getComponent(idxObj) + val);
			sigma.setElement(idxObj, idxObj + 1 + mean.dimension(), zMean.getComponent(idxObj) - val);
			for (int idx = 0; idx < mean.dimension(); idx++)
			{
				if (idx == idxObj)
					continue;
				sigma.setElement(idx, idxObj + 1, zMean.getComponent(idx));
				sigma.setElement(idx, idxObj + 1 + mean.dimension(), zMean.getComponent(idx));
			}
		}
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("zSigma: \r\n" + MatrixUtil.toString(sigma, 8, 5));

		sigma = usv[0].multiply(sigma);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigma: \r\n" + MatrixUtil.toString(sigma, 8, 5));
		return sigma;
	}

	
	public static ModelFilter getModelFilter()
	{
		return new EigenUKFModelFilter();
	}
}


class EigenUKFModelFilter implements ModelFilter
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

