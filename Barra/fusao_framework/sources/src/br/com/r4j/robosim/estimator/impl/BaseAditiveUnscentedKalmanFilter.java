package br.com.r4j.robosim.estimator.impl;

import java.util.ArrayList;
import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.EstimationException;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public abstract class BaseAditiveUnscentedKalmanFilter extends BaseEstimator
{
	private static Log log = LogFactory.getLog(BaseAditiveUnscentedKalmanFilter.class.getName());
	private static Log logTime = LogFactory.getLog("time");

	protected UKFDoubleVectorFunction stateFunction = null;

	protected double delta = 0;
	protected double gamma = 0;
	protected double w0state = 0;
	protected double w0cov = 0;
	protected double wi = 0;


	public BaseAditiveUnscentedKalmanFilter()
	{
		super();
	}


	public abstract String getName();


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		if (!(sensModel instanceof UKFDoubleVectorFunction))
			throw new ConfiguratorException("Sensor Model " + sensModel.getName() + " não implementa UKFDoubleVectorFunction");
		super.addSensorModel(sensModel, sens);
	}


	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof UKFDoubleVectorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " não implementa UKFDoubleVectorFunction");
		stateFunction = (UKFDoubleVectorFunction) dynModel;
		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	protected abstract void doEstimate();


	/**
	 */
	protected void setModifiers(int dim)
	{
		delta = 3 - dim;
		gamma = Math.sqrt(dim);
		w0state = 0;
		w0cov = 0;
		wi = 1.0/(2*dim);
	}


	protected AbstractDoubleMatrix generateSigmaPointsWithSqrtCovar(AbstractDoubleVector mean, AbstractDoubleMatrix covarSqrtUpper, double gamma)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));

		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			{
				double val = gamma*covarSqrtUpper.getElement(idxVar, idxObj);
				sigma.setElement(idxVar, idxObj + 1, mean.getComponent(idxVar) + val);
				sigma.setElement(idxVar, idxObj + 1 + mean.dimension(), mean.getComponent(idxVar) - val);
			}
		}

		return sigma;
	}


	public AbstractDoubleVector calculateMean(AbstractDoubleMatrix sigma, double w0state, double wi)
	{
		DoubleVector mean = new DoubleVector(sigma.rows());
		for (int idxState = 0; idxState < sigma.rows(); idxState++)
		{
			double val = w0state*sigma.getElement(idxState, 0);
			for (int i = 1; i < sigma.columns(); i++)
				val += wi*sigma.getElement(idxState, i);
			mean.setComponent(idxState, val);
		}
		return mean;
	}


	public AbstractDoubleSquareMatrix calculateCov(AbstractDoubleMatrix sigma, AbstractDoubleVector mean, double w0cov, double wi)
	{
		AbstractDoubleSquareMatrix cov = new DoubleSquareMatrix(mean.dimension());

		for (int i = 0; i < mean.dimension(); i++) for (int j = 0; j <= i; j++)
		{
			double val = w0cov*(sigma.getElement(i, 0) - mean.getComponent(i))*
							   (sigma.getElement(j, 0) - mean.getComponent(j));
			cov.setElement(i, j, val);
			cov.setElement(j, i, val);
		}

		AbstractDoubleSquareMatrix matrixTemp = new DoubleSquareMatrix(mean.dimension());
		for (int idxObjs = 0; idxObjs < mean.dimension(); idxObjs++)
		{
			for (int i = 0; i < mean.dimension(); i++) for (int j = 0; j <= i; j++)
			{
				double val = wi*(sigma.getElement(i, idxObjs + 1) - mean.getComponent(i))*
								(sigma.getElement(j, idxObjs + 1) - mean.getComponent(j)) +
							 wi*(sigma.getElement(i, idxObjs + mean.dimension() + 1) - mean.getComponent(i))*
								(sigma.getElement(j, idxObjs + mean.dimension() + 1) - mean.getComponent(j));
				matrixTemp.setElement(i, j, val);
				matrixTemp.setElement(j, i, val);
			}
			cov = cov.add(matrixTemp);
		}
		return cov;
	}


	public AbstractDoubleMatrix calculateCov(AbstractDoubleMatrix sigma_1, AbstractDoubleVector mean_1, AbstractDoubleMatrix sigma_2, AbstractDoubleVector mean_2, double w0cov, double wi)
	{
		AbstractDoubleMatrix cov = new DoubleMatrix(mean_1.dimension(), mean_2.dimension());
			
		for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
		{
			double val = w0cov*(sigma_1.getElement(i, 0) - mean_1.getComponent(i))*
							   (sigma_2.getElement(j, 0) - mean_2.getComponent(j));
			cov.setElement(i, j, val);
		}

		for (int idxObjs = 0; idxObjs < mean_1.dimension(); idxObjs++)
		{
			for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
			{
				double val = wi*(sigma_1.getElement(i, idxObjs + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + 1) - mean_2.getComponent(j)) + 
							 wi*(sigma_1.getElement(i, idxObjs + mean_1.dimension() + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + mean_2.dimension() + 1) - mean_2.getComponent(j));
				cov.setElement(i, j, cov.getElement(i, j) + val);
			}
		}
		return cov;
	}


	protected AbstractDoubleMatrix generateSigmaPoints(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar, double gamma)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		AbstractDoubleMatrix [] arraySqrts = null;
		double det = JSciMatrixMath.det(covar);
		log.debug("det: " + det);
		if (det == 0 || Math.abs(det) < 1E-9 || Double.isNaN(det))
		{
			int countIt = 0;
			double minDiag = MatrixUtil.getAbsMinValueOnDiagonal(covar);
			if (minDiag < 1E-6)
				minDiag = 1E-6;
			covar = MatrixUtil.clone(covar);
			while (det == 0 || Math.abs(det) < 1E-9 || Double.isNaN(det))
			{ 
				MatrixUtil.sumOnDiagonal(covar, minDiag);
				det = JSciMatrixMath.det(covar);
				log.debug("it: det: " + det);
				countIt++;
				if (countIt > 200)
				{
					log.debug("Kabum!!!");
					log.debug("covar: \r\n" + MatrixUtil.toString(covar, 9, 7));
					throw new EstimationException("UKF: Kabum!!!. countIt > 200. Não pode estimar.");
				}
			}
		}
		else if (det < 0)
		{
			log.debug("det < 0!!!");
			throw new EstimationException("UKF: det < 0!!!. Não pode estimar.");
		}
		arraySqrts = JSciMatrixMath.choleskyDecompose(covar);
		log.debug("arraySqrts[0]: \r\n" + MatrixUtil.toString(arraySqrts[0], 9, 7));
		log.debug("covar busted: \r\n" + MatrixUtil.toString(covar, 9, 4));
		
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));
		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			{
				double val = gamma*arraySqrts[0].getElement(idxVar, idxObj);
				sigma.setElement(idxVar, idxObj + 1, mean.getComponent(idxVar) + val);
				sigma.setElement(idxVar, idxObj + 1 + mean.dimension(), mean.getComponent(idxVar) - val);
			}
		}
		return sigma;
	}
}
