

package br.com.r4j.robosim.estimator.impl;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EstimationException;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public abstract class BaseAditiveUnscentedKalmanFilter extends BaseEstimator implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(BaseAditiveUnscentedKalmanFilter.class.getName());
	protected static Log logTime = LogFactory.getLog("time");

	protected Log logEstimator = null;

	protected UKFDoubleVectorFunction stateFunction = null;

	protected double alpha = 0;
	protected double beta = 0;
	protected double delta = 0;
	protected double kappa = 0;

	protected double gamma = 0;
	protected double gammaSqrt = 0;

	protected double w0state = 0;
	protected double w0cov = 0;
	protected double wi = 0;
	protected double wiCov = 0;
	
	private Shape shpBase = null;
	private Shape shpBase2 = null;
	

	public BaseAditiveUnscentedKalmanFilter()
	{
		super();
		shpBase = ShapesUtil.createAlvo(4, 6, 9);
		shpBase2 = ShapesUtil.createSixPointsStart(12, 4);
	}


	public abstract String getName();


	public List getRenderers()
	{
		ArrayList list = new ArrayList();
		list.add(this);
		return list;
	}


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
/*
	protected void setModifiers(int dim)
	{
		alpha = 1; // 1e-4
		beta = 2;
		kappa = 2 - dim;
//		delta = 2; // 3 - dim
		delta = alpha*alpha*(dim + kappa) - dim; // 3 - dim

		gamma = dim + delta;
		gammaSqrt = Math.sqrt(Math.abs(gamma));
		w0state = delta/(dim + delta);
		w0cov = delta/gamma + (1 - alpha*alpha + beta);
		wi = 1.0/(2*(dim + delta));
		wiCov = 1.0/(2*(dim + delta));

		logEstimator.debug("NHÉ: delta: " + delta + ", dim: " + dim + ", gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
		
	}
//*/


	protected void setModifiers(int dim)
	{
		alpha = 1; // 1e-4
		beta = 2;
//		beta = 0;
		kappa = 3 - dim;
		delta = 2; // 3 - dim
//		delta = alpha*alpha*(dim + kappa) - dim; // 3 - dim
//		delta = 3 - dim;

		gamma = dim + delta;
		gammaSqrt = Math.sqrt(Math.abs(gamma));

		w0cov = delta/(dim + delta);
		wiCov = 1.0/(2*(dim + delta));

		w0state = delta/(dim + delta);
		wi = 1.0/(2*(dim + delta));

		if (logEstimator.isDebugEnabled())
			logEstimator.debug("delta: " + delta + ", dim: " + dim + ", gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
	}


	protected void setVLinesStateModifiers(int dim)
	{
		alpha = 1; // 1e-4
//		beta = 2;
		beta = 0;
		kappa = 3 - dim;
//		delta = 2; // 3 - dim
		delta = alpha*alpha*(dim + kappa) - dim; // 3 - dim
//		delta = 3 - dim;

		double gammaAtt = 10;
		delta = 1;
//		gamma = 0.7;
		gamma = dim + delta;
		gammaSqrt = Math.sqrt(Math.abs(gamma))/gammaAtt;
//		w0cov = 1 + beta + delta/(dim + delta);
		w0cov = delta/(dim + delta)*gammaAtt*gammaAtt;
		wiCov = 1.0/(2*(dim + delta))*gammaAtt*gammaAtt;

		delta = 10;
		w0state = delta/(dim + delta);
		wi = 1.0/(2*(dim + delta));

		if (logEstimator.isDebugEnabled())
			logEstimator.debug("line state: delta: " + delta + ", dim: " + dim + ", gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
	}


/*
	public AbstractDoubleVector calculateMean(AbstractDoubleMatrix sigma)
	{
		logEstimator.debug("mean: delta: " + delta + ", gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
		logEstimator.debug("sigma.rows(): " + sigma.rows() + ", sigma.columns(): " + sigma.columns());
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
/*/

	public AbstractDoubleVector calculateMean(AbstractDoubleMatrix sigma)
	{
		AbstractDoubleMatrix sigmaTmp = MatrixUtil.clone(sigma);
/*
		for (int idxState = 0; idxState < sigma.rows(); idxState++)
			for (int i = 1; i < sigma.columns(); i++)
				if ((i - 1)%sigma.rows() != idxState)
					sigmaTmp.setElement(idxState, i, sigma.getElement(idxState, 0));
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigmaTmp: \r\n" + MatrixUtil.toString(sigmaTmp, 7, 3));
//*/
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("mean: delta: " + delta + ", gamma: " + gamma + ", gammaSqrt: " + gammaSqrt + ", w0state: " + w0state + ", wi: " + wi + ", w0cov: " + w0cov + ", wiCov: " + wiCov);
			logEstimator.debug("sigma.rows(): " + sigma.rows() + ", sigma.columns(): " + sigma.columns());
		}
		StringBuffer strBuff = new StringBuffer();
		StringBuffer strBuff2 = new StringBuffer();
		DoubleVector mean = new DoubleVector(sigma.rows());
		for (int idxState = 0; idxState < sigma.rows(); idxState++)
		{
			strBuff.append(sigma.getElement(idxState, 0));
			strBuff2.append(w0state*sigma.getElement(idxState, 0));
			double val = w0state*sigma.getElement(idxState, 0);
			for (int i = 1; i < sigma.columns(); i++)
			{
				val += wi*sigma.getElement(idxState, i);
				strBuff.append("|");
				strBuff2.append("|");
				strBuff.append(sigma.getElement(idxState, i));
				strBuff2.append(wi*sigma.getElement(idxState, i));
			}
//			logEstimator.debug("mean vals: " + strBuff);
//			logEstimator.debug("sigm vals: " + strBuff2);
			mean.setComponent(idxState, val);
		}
		return mean;
	}
//*/


	public AbstractDoubleSquareMatrix calculateCov(AbstractDoubleMatrix sigma, AbstractDoubleVector mean)
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
		for (int idxObjs = 0; idxObjs < (sigma.columns() - 1)/2; idxObjs++)
		{
			for (int i = 0; i < mean.dimension(); i++) for (int j = 0; j <= i; j++)
			{
				double val = wiCov*(sigma.getElement(i, idxObjs + 1) - mean.getComponent(i))*
								(sigma.getElement(j, idxObjs + 1) - mean.getComponent(j)) +
							 wiCov*(sigma.getElement(i, idxObjs + (sigma.columns() - 1)/2 + 1) - mean.getComponent(i))*
								(sigma.getElement(j, idxObjs + (sigma.columns() - 1)/2 + 1) - mean.getComponent(j));
				matrixTemp.setElement(i, j, val);
				matrixTemp.setElement(j, i, val);
			}
			cov = cov.add(matrixTemp);
		}
		return cov;
	}


	public AbstractDoubleMatrix calculateCov(AbstractDoubleMatrix sigma_1, AbstractDoubleVector mean_1, AbstractDoubleMatrix sigma_2, AbstractDoubleVector mean_2)
	{
		AbstractDoubleMatrix cov = new DoubleMatrix(mean_1.dimension(), mean_2.dimension());
			
		for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
		{
			double val = w0cov*(sigma_1.getElement(i, 0) - mean_1.getComponent(i))*
							   (sigma_2.getElement(j, 0) - mean_2.getComponent(j));
			cov.setElement(i, j, val);
		}

		for (int idxObjs = 0; idxObjs < (sigma_1.columns() - 1)/2; idxObjs++)
		{
			for (int i = 0; i < mean_1.dimension(); i++) for (int j = 0; j < mean_2.dimension(); j++)
			{
				double val = wiCov*(sigma_1.getElement(i, idxObjs + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + 1) - mean_2.getComponent(j)) + 
							 wiCov*(sigma_1.getElement(i, idxObjs + (sigma_1.columns() - 1)/2 + 1) - mean_1.getComponent(i))*
								(sigma_2.getElement(j, idxObjs + (sigma_1.columns() - 1)/2 + 1) - mean_2.getComponent(j));
				cov.setElement(i, j, cov.getElement(i, j) + val);
			}
		}
		return cov;
	}


	protected AbstractDoubleMatrix generateSigmaPoints(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);

		AbstractDoubleMatrix [] arraySqrts = null;
		double det = JSciMatrixMath.det(covar);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("det: " + det);
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
				if (logEstimator.isDebugEnabled())
					logEstimator.debug("it: det: " + det);
				countIt++;
				if (countIt > 200)
				{
					logEstimator.warn("Kabum!!!");
					logEstimator.warn("covar: \r\n" + MatrixUtil.toString(covar, 9, 7));
					throw new EstimationException("UKF: Kabum!!!. countIt > 200. Não pode estimar.");
				}
			}
		}
		else if (det < 0)
		{
			logEstimator.warn("det < 0!!!");
			throw new EstimationException("UKF: det < 0!!!. Não pode estimar.");
		}

		arraySqrts = JSciMatrixMath.choleskyDecompose(covar);
		if (logEstimator.isDebugEnabled())
		{
//			logEstimator.debug("arraySqrts[0]: \r\n" + MatrixUtil.toString(arraySqrts[0], 7, 6));
			logEstimator.debug("covar busted: \r\n" + MatrixUtil.toString(covar, 9, 4));
		}
		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));
		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			{
				double val = gammaSqrt*arraySqrts[0].getElement(idxVar, idxObj);
				sigma.setElement(idxVar, idxObj + 1, mean.getComponent(idxVar) + val);
				sigma.setElement(idxVar, idxObj + 1 + mean.dimension(), mean.getComponent(idxVar) - val);
			}
		}
//		if (logEstimator.isDebugEnabled())
//			logEstimator.debug("sigma: \r\n" + MatrixUtil.toString(sigma, 8, 5));
		return sigma;
	}


	protected AbstractDoubleMatrix generateSigmaPointsWithSqrtCovar(AbstractDoubleVector mean, AbstractDoubleMatrix covarSqrtUpper)
	{
		DoubleMatrix sigma = new DoubleMatrix(mean.dimension(), mean.dimension()*2 + 1);
		
		AbstractDoubleMatrix covarSqrtLower = (AbstractDoubleMatrix) covarSqrtUpper.transpose();
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("covarSqrtLower: \r\n" + MatrixUtil.toString(covarSqrtLower, 9, 7));

		for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			sigma.setElement(idxVar, 0, mean.getComponent(idxVar));

		for (int idxObj = 0; idxObj < mean.dimension(); idxObj++)
		{
			for (int idxVar = 0; idxVar < mean.dimension(); idxVar++)
			{
				double val = gammaSqrt*covarSqrtLower.getElement(idxVar, idxObj);
				sigma.setElement(idxVar, idxObj + 1, mean.getComponent(idxVar) + val);
				sigma.setElement(idxVar, idxObj + 1 + mean.dimension(), mean.getComponent(idxVar) - val);
			}
		}

		return sigma;
	}


	private int currentStep = -1;
	private ArrayList listSigmasPred = new ArrayList();
	private ArrayList listSigmasFilt = new ArrayList();
	private boolean bRendering = false;
	private EstimatorRendererInfo rndrInfo = null;
	public void setEstimatorRendererInfo(EstimatorRendererInfo info) {rndrInfo = info;}
	public void setStep(int currentStep) {this.currentStep = currentStep;}
	public void newPose(Pose2D pose) {currentStep++;}
	public void addSigmaPointsPredicted(AbstractDoubleMatrix sigmaPoints) {listSigmasPred.add(sigmaPoints);}
	public void addSigmaPointsFiltered(AbstractDoubleMatrix sigmaPoints) {listSigmasFilt.add(sigmaPoints);}
	public void noSigmaPointsFiltered() {listSigmasFilt.add("xu");}
	public void imageUpdatePerformed(RendererEvent e) {}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (currentStep >= 0 && listSigmasPred.size() > 0)
		{
			AbstractDoubleMatrix sigmaPointsPred = (AbstractDoubleMatrix) listSigmasPred.get(currentStep);
			Graphics2D g2d = e.getGraphics();
			g2d.setPaintMode();

			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(rndrInfo.getColorFill());

			for (int i = 0; i < sigmaPointsPred.columns(); i++)
			{
				double xCenter = sigmaPointsPred.getElement(0, i);  
				double yCenter = sigmaPointsPred.getElement(1, i);
				
				Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase);
				g2d.draw(shpShp);
			}	

			Object oSigmaPointsFilt = listSigmasFilt.get(currentStep);
			if (oSigmaPointsFilt instanceof AbstractDoubleMatrix)
			{
				AbstractDoubleMatrix sigmaPointsFilt = (AbstractDoubleMatrix) oSigmaPointsFilt;
				for (int i = 0; i < sigmaPointsFilt.columns(); i++)
				{
					double xCenter = sigmaPointsFilt.getElement(0, i);  
					double yCenter = sigmaPointsFilt.getElement(1, i);
				
					Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase2);
					g2d.draw(shpShp);
				}	
			}
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		if (currentStep >= 0 && listSigmasPred.size() > 0)
		{
			AbstractDoubleMatrix sigmaPoints = (AbstractDoubleMatrix) listSigmasPred.get(currentStep);
			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);

			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(rndrInfo.getColorFill());

			for (int i = 0; i < sigmaPoints.columns(); i++)
			{
				double xCenter = sigmaPoints.getElement(0, i);  
				double yCenter = sigmaPoints.getElement(1, i);
				
				Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase);
				g2d.draw(shpShp);
			}	

			Object oSigmaPointsFilt = listSigmasFilt.get(currentStep);
			if (oSigmaPointsFilt instanceof AbstractDoubleMatrix)
			{
				AbstractDoubleMatrix sigmaPointsFilt = (AbstractDoubleMatrix) oSigmaPointsFilt;
				for (int i = 0; i < sigmaPointsFilt.columns(); i++)
				{
					double xCenter = sigmaPointsFilt.getElement(0, i);  
					double yCenter = sigmaPointsFilt.getElement(1, i);
				
					Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase2);
					g2d.draw(shpShp);
				}	
			}
		}
		bRendering = false;
	}


	public void erase(RendererEvent e)
	{
		bRendering = true;
		if (currentStep >= 0 && listSigmasPred.size() > 0)
		{
			AbstractDoubleMatrix sigmaPoints = (AbstractDoubleMatrix) listSigmasPred.get(currentStep);
			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);

			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(rndrInfo.getColorFill());

			for (int i = 0; i < sigmaPoints.columns(); i++)
			{
				double xCenter = sigmaPoints.getElement(0, i);  
				double yCenter = sigmaPoints.getElement(1, i);
				
				Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase);
				g2d.draw(shpShp);
			}	

			Object oSigmaPointsFilt = listSigmasFilt.get(currentStep);
			if (oSigmaPointsFilt instanceof AbstractDoubleMatrix)
			{
				AbstractDoubleMatrix sigmaPointsFilt = (AbstractDoubleMatrix) oSigmaPointsFilt;
				for (int i = 0; i < sigmaPointsFilt.columns(); i++)
				{
					double xCenter = sigmaPointsFilt.getElement(0, i);  
					double yCenter = sigmaPointsFilt.getElement(1, i);
				
					Shape shpShp = e.translateAndMaintainShapeSize(xCenter, yCenter, shpBase2);
					g2d.draw(shpShp);
				}	
			}
		}
		bRendering = false;
	}
}
