package br.com.r4j.math.kalmanfilter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;


public class AditiveNosieUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("ukf");
	private static Log logUKF_I = LogFactory.getLog("ukf_input");
	private static Log logUKF_covar = LogFactory.getLog("ukf_covar");

	
	private boolean bStateReset = false;

	private AbstractDoubleVector stateMeasures = null;
	private AbstractDoubleSquareMatrix stateCovarMeasures = null;

/*
	private AbstractDoubleVector stateLast = null;
	private AbstractDoubleSquareMatrix stateCovarLast = null;
//*/
	private AbstractDoubleSquareMatrix stateCovarAditive = null;

	private DoubleVectorFunction stateFunction = null;
	private DoubleVectorFunction measureFunction = null;

	private AbstractDoubleVector stateEstimate = null;
	private AbstractDoubleSquareMatrix stateCovarEstimate = null;


	public AditiveNosieUnscentedKalmanFilter()
	{
	}


	public void setMeasure(AbstractDoubleVector stateMeasures, AbstractDoubleSquareMatrix stateCovarMeasures)
	{
		this.stateMeasures = stateMeasures;
		this.stateCovarMeasures = stateCovarMeasures;
	}


	public void setLastState(AbstractDoubleVector stateLast, AbstractDoubleSquareMatrix stateCovarLast, AbstractDoubleSquareMatrix stateRobotCovarInc)
	{
		this.bStateReset = true;
		this.stateEstimate = stateLast;
		this.stateCovarEstimate = stateCovarLast;
		this.stateCovarAditive = stateRobotCovarInc;
	}


	public void setStateTransitionErrorInc(AbstractDoubleSquareMatrix stateRobotCovarInc)
	{
		this.bStateReset = true;
		this.stateCovarAditive = stateRobotCovarInc;
	}


	public void setStateTransitionFunction(DoubleVectorFunction stateFunction)
	{
		this.stateFunction = stateFunction;
	}


	public void setMeasureEstimationFunction(DoubleVectorFunction measureFunction)
	{
		this.measureFunction = measureFunction;
	}


	public void update()
	{
		if (!bStateReset)
		{
			stateCovarAditive = new DoubleSquareMatrix(stateCovarEstimate.rows());
		}

		logUKF.debug("------------------------------ NOVA ITER");
		logUKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
		logUKF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
		if (stateMeasures != null)
		{
			logUKF.debug("stateMeasures: \r\n" + MatrixUtil.toString(stateMeasures, 9, 4));
			logUKF.debug("stateCovarMeasures: \r\n" + MatrixUtil.toString(stateCovarMeasures, 9, 4));
		}
		logUKF.debug("stateCovarAditive: \r\n" + MatrixUtil.toString(stateCovarAditive, 9, 4));
		logUKF.debug("stateCovarEstimate.isSymmetric(): " + stateCovarEstimate.isSymmetric() + ", stateCovarEstimate.det(): " + stateCovarEstimate.det());
		logUKF_I.debug("------------------------------ NOVA ITER");
		logUKF_I.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
		logUKF_I.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
		logUKF_I.debug("stateCovarAditive: \r\n" + MatrixUtil.toString(stateCovarAditive, 9, 4));

		if (stateMeasures != null)
		{
			logUKF_I.debug("stateMeasures: \r\n" + MatrixUtil.toString(stateMeasures, 9, 4));
			logUKF_I.debug("stateCovarMeasures: \r\n" + MatrixUtil.toString(stateCovarMeasures, 9, 4));
		}
		logUKF_I.debug("stateCovarEstimate.isSymmetric(): " + stateCovarEstimate.isSymmetric() + ", stateCovarEstimate.det(): " + stateCovarEstimate.det());

		logUKF_covar.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
		logUKF_covar.debug("stateCovarEstimate.isSymmetric(): " + stateCovarEstimate.isSymmetric() + ", stateCovarEstimate.det(): " + stateCovarEstimate.det());

		// 1 - Calcula os sigma points
		//
//		logUKF.debug("arrayCovarSqrts[0]: \r\n" + MatrixUtil.toString(arrayCovarSqrts[0], 9, 4));
//		double alpha = 0.001, ka = 0, beta = 2;
		double delta = 3 - stateEstimate.dimension();//2; //alpha*alpha*(stateEstimate.dimension() + ka) - stateEstimate.dimension();
		double gamma = Math.sqrt(stateEstimate.dimension() + delta);
		double w0state = delta/(delta + stateEstimate.dimension());
		double w0cov = w0state;// + (1 - alpha*alpha + beta);
		double wi = 1.0/(delta + stateEstimate.dimension())/2;
		logUKF.debug("w0state = " + w0state + ", w0cov = " + w0cov + ", wi = " + wi + ", gamma: " + gamma + ", delta = " + delta);

		// Calcula os sigmas iniciais
		AbstractDoubleMatrix stateLastSigma = this.generateSigmaPoints(stateEstimate, stateCovarEstimate, gamma);
//		logUKF.debug("stateLastSigma: \r\n" + MatrixUtil.toString(stateLastSigma, 9, 4));

		// Calcula os sigmas pela tranforma��o de estado
		AbstractDoubleMatrix stateSigma = new DoubleMatrix(stateEstimate.dimension(), stateEstimate.dimension()*2 + 1);
		stateFunction.calculate(stateLastSigma, stateSigma);
//		logUKF.debug("stateSigma: \r\n" + MatrixUtil.toString(stateSigma, 9, 4));


		// 2 - Calcula os vetores stateFromTransition, measureFromEstimation e stateCovarFrom
		//
		AbstractDoubleVector stateFromTransition = this.calculateMean(stateSigma, w0state, wi);
		logUKF.debug("stateFromTransition: \r\n" + MatrixUtil.toString(stateFromTransition, 9, 4));

		AbstractDoubleSquareMatrix pXX = this.calculateCov(stateSigma, stateFromTransition, w0cov, wi);
		pXX = pXX.add(stateCovarAditive);
		logUKF.debug("pXX: \r\n" + MatrixUtil.toString(pXX, 9, 4));
		logUKF.debug("pXX.isSymmetric(): " + pXX.isSymmetric() + ", pXX.det(): " + pXX.det());

		// 3 - calcula stateEstimate e stateCovarEstimate
		//
		if (stateMeasures != null && stateMeasures.dimension() > 0)
		{
			// Calcula os sigmas pela tranforma��o de medi��o
			DoubleMatrix measSigma = new DoubleMatrix(stateMeasures.dimension(), stateEstimate.dimension()*2 + 1);
			measureFunction.calculate(stateSigma, measSigma);
//				logUKF.debug("measSigma: \r\n" + MatrixUtil.toString(measSigma, 9, 4));

			AbstractDoubleVector measureFromEstimation = this.calculateMean(measSigma, w0state, wi);
//			logUKF.debug("measureFromEstimation: \r\n" + MatrixUtil.toString(measureFromEstimation, 9, 4));

			AbstractDoubleSquareMatrix pYY = null;
			AbstractDoubleMatrix pXY = null;
//*
			AbstractDoubleMatrix stateNewSigma = this.generateSigmaPoints(stateFromTransition, pXX, gamma);

			// Calcula os sigmas pela tranforma��o de medi��o
			AbstractDoubleMatrix measNewSigma = new DoubleMatrix(stateMeasures.dimension(), stateEstimate.dimension()*2 + 1);
			measureFunction.calculate(stateNewSigma, measNewSigma);

			AbstractDoubleVector stateFromTransitionNew = this.calculateMean(stateNewSigma, w0state, wi);
			AbstractDoubleVector measureFromEstimationNew = this.calculateMean(measNewSigma, w0state, wi);
			logUKF.debug("stateNewSigma: \r\n" + MatrixUtil.toString(stateNewSigma, 9, 4));
			logUKF.debug("measNewSigma: \r\n" + MatrixUtil.toString(measNewSigma, 9, 4));
			logUKF.debug("stateFromTransitionNew: \r\n" + MatrixUtil.toString(stateFromTransitionNew, 9, 4));
			logUKF.debug("measureFromEstimationNew: \r\n" + MatrixUtil.toString(measureFromEstimationNew, 9, 4));

			pXY = this.calculateCov(stateSigma, stateFromTransition, measSigma, stateMeasures, w0cov, wi);
//			pXY = this.calculateCov(stateNewSigma, stateFromTransitionNew, measNewSigma, measureFromEstimationNew, w0cov, wi);
			pYY = this.calculateCov(measNewSigma, measureFromEstimationNew, w0cov, wi);
			pYY = pYY.add(stateCovarMeasures);
/*/
			pXY = this.calculateCov(stateSigma, stateFromTransition, measSigma, stateMeasures, w0cov, wi);
			pYY = this.calculateCov(measSigma, measureFromEstimation, w0cov, wi);
			pYY = pYY.add(stateCovarMeasures);
//*/
			AbstractDoubleSquareMatrix pYYInv = pYY.inverse();

			logUKF.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 9, 4));
			logUKF.debug("pYY: \r\n" + MatrixUtil.toString(pYY, 9, 4));
			logUKF.debug("pYY.isSymmetric(): " + pYY.isSymmetric() + ", pYY.det(): " + pYY.det());
//			logUKF.debug("pYYInv: \r\n" + MatrixUtil.toString(pYYInv, 9, 4));

			AbstractDoubleMatrix gain = pXY.multiply(pYYInv);
			logUKF.debug("gain: \r\n" + MatrixUtil.toString(gain, 9, 4));
////			AbstractDoubleVector measDiff = stateMeasures.subtract(measureFromEstimation);
			AbstractDoubleVector measDiff = stateMeasures.subtract(measureFromEstimationNew);
			logUKF.debug("measDiff: \r\n" + MatrixUtil.toString(measDiff, 9, 4));
			AbstractDoubleVector stateGain =  gain.multiply(measDiff);
			logUKF.debug("stateGain: \r\n" + MatrixUtil.toString(stateGain, 9, 4));

////			stateEstimate = stateFromTransition.add(stateGain);
			stateEstimate = stateFromTransitionNew.add(stateGain);
////			stateEstimate = stateFromTransitionNew.subtract(stateGain);

			logUKF.debug("pXX: \r\n" + MatrixUtil.toString(pXX, 9, 4));
//			AbstractDoubleSquareMatrix [] pXXUSV = pXX.singularValueDecompose();
//			logUKF.debug("pXXUSV[1]: \r\n" + MatrixUtil.toString(pXXUSV[1], 7, 7));
			logUKF.debug("pXX.isSymmetric(): " + pXX.isSymmetric() + ", pXX.det(): " + pXX.det());

			AbstractDoubleMatrix gain_x_pYY = gain.multiply(pYY);
			AbstractDoubleMatrix gain_x_pYY_x_gainT = gain_x_pYY.multiply((AbstractDoubleMatrix) gain.transpose());
//			logUKF.debug("gain_x_pYY: \r\n" + MatrixUtil.toString(gain_x_pYY, 9, 4));
			logUKF.debug("gain_x_pYY_x_gainT: \r\n" + MatrixUtil.toString(gain_x_pYY_x_gainT, 9, 4));
			AbstractDoubleSquareMatrix pXXcorrSim = MatrixUtil.convert2SquareMatrix(gain_x_pYY_x_gainT);
//			AbstractDoubleSquareMatrix [] pXXUSVcorrSim = pXXcorrSim.singularValueDecompose();
//			logUKF.debug("pXXUSVcorrSim[1]: \r\n" + MatrixUtil.toString(pXXUSVcorrSim[1], 7, 7));
			logUKF.debug("pXXcorr.isSymmetric(): " + pXXcorrSim.isSymmetric() + ", pXXcorr.det(): " + pXXcorrSim.det());

			AbstractDoubleMatrix gain_x_pXYT = gain.multiply((AbstractDoubleMatrix) pXY.transpose());
			AbstractDoubleMatrix pXY_x_gainT = (AbstractDoubleMatrix) gain_x_pXYT.transpose();
			AbstractDoubleMatrix pXY_x_gainT_2 = pXY_x_gainT.add(gain_x_pXYT);

			logUKF_covar.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 9, 4));
			logUKF_covar.debug("pYY: \r\n" + MatrixUtil.toString(pYY, 9, 4));
			logUKF_covar.debug("gain: \r\n" + MatrixUtil.toString(gain, 9, 4));
			logUKF_covar.debug("pXX: \r\n" + MatrixUtil.toString(pXX, 9, 4));
			logUKF_covar.debug("gain_x_pYY_x_gainT: \r\n" + MatrixUtil.toString(gain_x_pYY_x_gainT, 9, 4));
			logUKF_covar.debug("pXY_x_gainT: \r\n" + MatrixUtil.toString(pXY_x_gainT, 9, 4));
			logUKF_covar.debug("pXY_x_gainT_2: \r\n" + MatrixUtil.toString(pXY_x_gainT_2, 9, 4));

			
//			stateCovarEstimate = MatrixUtil.convert2SquareMatrix(pXX.subtract(pXXcorrSim));
			stateCovarEstimate = MatrixUtil.convert2SquareMatrix(pXX.add(pXXcorrSim).subtract(gain_x_pXYT).subtract(pXY_x_gainT));
/*
			logUKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
			logUKF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
			logUKF.debug("stateCovarEstimate.isSymmetric(): " + stateCovarEstimate.isSymmetric() + ", stateCovarEstimate.det(): " + stateCovarEstimate.det());
//*/
		}
		else
		{
			stateEstimate = stateFromTransition;
			stateCovarEstimate = pXX;
		}


		// 4 - Finaliza. Finalize j�, Finalize j�!
		//
		bStateReset = false;
	}


	public AbstractDoubleVector getEstimateExpectancy()
	{
		return stateEstimate;
	}


	public AbstractDoubleSquareMatrix getEstimateCovariance()
	{
		return stateCovarEstimate;
	}



	public AbstractDoubleMatrix generateSigmaPoints(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar, double gamma)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		AbstractDoubleSquareMatrix [] arraySqrts = covar.choleskyDecompose();
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
		{
			for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
			{
				// Invertido para pegar o valor das linhas da raiz para as colunas dos sigmas.
////				double val = gamma*arraySqrts[0].getElement(idxObj, idxVar);
				double val = gamma*arraySqrts[0].getElement(idxVar, idxObj);
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

		AbstractDoubleSquareMatrix matrixTemp = new DoubleSquareMatrix(mean.dimension());
		for (int i = 0; i < mean.dimension(); i++) for (int j = 0; j <= i; j++)
		{
			double val = w0cov*(sigma.getElement(i, 0) - mean.getComponent(i))*
						       (sigma.getElement(j, 0) - mean.getComponent(j));
			matrixTemp.setElement(i, j, val);
			matrixTemp.setElement(j, i, val);
		}
		cov = cov.add(matrixTemp);
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
			
		AbstractDoubleMatrix matrixTemp = new DoubleMatrix(mean_1.dimension(), mean_2.dimension());
		for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
		{
			double val = w0cov*(sigma_1.getElement(i, 0) - mean_1.getComponent(i))*
							   (sigma_2.getElement(j, 0) - mean_2.getComponent(j));
			matrixTemp.setElement(i, j, val);
		}
		cov = cov.add(matrixTemp);
		for (int idxObjs = 0; idxObjs < mean_1.dimension(); idxObjs++)
		{
			for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
			{
				double val = wi*(sigma_1.getElement(i, idxObjs + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + 1) - mean_2.getComponent(j)) + 
				             wi*(sigma_1.getElement(i, idxObjs + mean_1.dimension() + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + mean_1.dimension() + 1) - mean_2.getComponent(j));
				matrixTemp.setElement(i, j, val);
			}
			cov = cov.add(matrixTemp);
		}
		return cov;
	}
}

