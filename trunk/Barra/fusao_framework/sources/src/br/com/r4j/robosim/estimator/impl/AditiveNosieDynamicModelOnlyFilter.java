package br.com.r4j.robosim.estimator.impl;

import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;



public class AditiveNosieDynamicModelOnlyFilter extends BaseEstimator // implements AditiveNosieStatisticalFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieDynamicModelOnlyFilter.class.getName());
	private static Log logMov = LogFactory.getLog("movfilter");
	private static Log logTime = LogFactory.getLog("time");


	private EKFDoubleVectorFunction ekfFunc = null;


	public AditiveNosieDynamicModelOnlyFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Movimento";
	}


	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " não implementa EKFDoubleVectorFunction");

		this.ekfFunc = (EKFDoubleVectorFunction) dynModel;

		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	protected void doEstimate()
	{
		logMov.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
		logMov.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
		logMov.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));

		// predição
		AbstractDoubleVector statei_iminus1 = this.estimateStateMean();
		AbstractDoubleMatrix Fi = ekfFunc.getTransitionMatrixJacobian(this.getMean());

		logMov.debug("Fi: \r\n" + MatrixUtil.toString(Fi, 9, 4));

		AbstractDoubleSquareMatrix Pi_iminus1 = this.estimateStateCovar(Fi);
		this.setPredictedState(statei_iminus1, Pi_iminus1);
	}

	
	private AbstractDoubleVector estimateStateMean()
	{
		AbstractDoubleVector vectReadings = new DoubleVector(dynModelSensor.getDataDimension());
		AbstractDoubleVector vectEst = new DoubleVector(dynModel.getDataDimension());
		dynModelSensor.getData(vectReadings);
		dynModel.produceResults(this.getMean(), vectReadings, this.getCovariance(), vectEst);

		return vectEst;
	}


	private AbstractDoubleSquareMatrix estimateStateCovar(AbstractDoubleMatrix Fi)
	{
		AbstractDoubleMatrix FiT = (AbstractDoubleMatrix) Fi.transpose();

		AbstractDoubleMatrix tmp1 = Fi.multiply(this.getCovariance()).multiply(FiT);
		logMov.debug("this.getCovariance(): \r\n" + MatrixUtil.toString(this.getCovariance(), 9, 7));
		logMov.debug("F.P.FT: \r\n" + MatrixUtil.toString(tmp1, 9, 7));
		
		AbstractDoubleSquareMatrix senQ = dynModelSensor.getDataCovariance(); 
		AbstractDoubleMatrix Q = dynModel.getModelIncrementalCovariance(null, null, senQ); 
		tmp1 = tmp1.add(Q);

		logMov.debug("senQ: \r\n" + MatrixUtil.toString(senQ, 9, 7));
		logMov.debug("Q: \r\n" + MatrixUtil.toString(Q, 9, 7));
		logMov.debug("F.P.FT+Q: \r\n" + MatrixUtil.toString(tmp1, 9, 7));

		return MatrixUtil.convert2SquareMatrix(tmp1);
	}


	public static ModelFilter getModelFilter()
	{
		return new DynModelModelFilter();
	}
}


class DynModelModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return false;
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return false;
	}
}

