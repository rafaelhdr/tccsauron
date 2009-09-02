package br.com.r4j.robosim.estimator.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;



/** @modelguid {9AF26DF1-D929-4BF7-A91E-18FE297A24A2} */
public class AditiveNosieDynamicModelOnlyFilter extends BaseEstimator // implements AditiveNosieStatisticalFilter
{
	/** @modelguid {9D146DA4-D0E1-441C-88A6-993CE434363A} */
	private static Log log = LogFactory.getLog(AditiveNosieDynamicModelOnlyFilter.class.getName());
	/** @modelguid {F40997D4-C749-4BDE-910D-428CDE243121} */
	private static Log logMov = LogFactory.getLog("movfilter");
	/** @modelguid {3E93B3EC-89ED-4B3C-805D-501A21BDAFAA} */
	private static Log logTime = LogFactory.getLog("time");


	/** @modelguid {0C2E0385-868B-4E24-B50E-DD0A61828485} */
	private EKFDoubleVectorFunction ekfFunc = null;


	/** @modelguid {12A5CF1A-D183-41F2-92D2-EC0CDB09E47B} */
	public AditiveNosieDynamicModelOnlyFilter()
	{
		super();
	}


	/** @modelguid {419AADFA-17C0-4358-A41B-7D9EBB50146E} */
	public String getName()
	{
		return "Filtro de Movimento";
	}


	/** @modelguid {C0513DDD-1ED5-4C5A-8B6E-C82D76944966} */
	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " não implementa EKFDoubleVectorFunction");

		this.ekfFunc = (EKFDoubleVectorFunction) dynModel;

		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	/** @modelguid {C4004B5D-A1B5-44CC-A9CF-8F566090A00B} */
	protected void doEstimate()
	{
		if (logMov.isDebugEnabled())
		{
			logMov.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
			logMov.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
			logMov.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
		}

		// predição
		AbstractDoubleVector statei_iminus1 = this.estimateStateMean();
		AbstractDoubleMatrix Fi = ekfFunc.getTransitionMatrixJacobian(this.getMean());

		if (logMov.isDebugEnabled())
			logMov.debug("Fi: \r\n" + MatrixUtil.toString(Fi, 9, 4));

		AbstractDoubleSquareMatrix Pi_iminus1 = this.estimateStateCovar(Fi);
		this.setPredictedState(statei_iminus1, Pi_iminus1);
	}

	
	/** @modelguid {24408B28-EC0F-488D-AE9A-3026A84915D8} */
	private AbstractDoubleVector estimateStateMean()
	{
		AbstractDoubleVector vectReadings = new DoubleVector(dynModelSensor.getDataDimension(dynModel));
		dynModelSensor.getData(vectReadings, dynModel);
		return dynModel.produceResults(this.getMean(), vectReadings, this.getCovariance());
	}


	/** @modelguid {E6ED9094-CA16-4EF5-B8FB-8B5CFC2E6F63} */
	private AbstractDoubleSquareMatrix estimateStateCovar(AbstractDoubleMatrix Fi)
	{
		AbstractDoubleMatrix FiT = (AbstractDoubleMatrix) Fi.transpose();

		AbstractDoubleMatrix tmp1 = Fi.multiply(this.getCovariance()).multiply(FiT);
		if (logMov.isDebugEnabled())
		{
			logMov.debug("this.getCovariance(): \r\n" + MatrixUtil.toString(this.getCovariance(), 9, 7));
			logMov.debug("F.P.FT: \r\n" + MatrixUtil.toString(tmp1, 9, 7));
		}
		
		AbstractDoubleSquareMatrix senQ = dynModelSensor.getDataCovariance(dynModel); 
		AbstractDoubleMatrix Q = dynModel.getModelIncrementalCovariance(null, null, senQ); 
		tmp1 = tmp1.add(Q);

		if (logMov.isDebugEnabled())
		{
			logMov.debug("senQ: \r\n" + MatrixUtil.toString(senQ, 9, 7));
			logMov.debug("Q: \r\n" + MatrixUtil.toString(Q, 9, 7));
			logMov.debug("F.P.FT+Q: \r\n" + MatrixUtil.toString(tmp1, 9, 7));
		}

		return MatrixUtil.convert2SquareMatrix(tmp1);
	}


	/** @modelguid {43ECB55A-C36E-4E22-8C54-D3615365B4BC} */
	public static ModelFilter getModelFilter()
	{
		return new DynModelModelFilter();
	}
}


/** @modelguid {A1B92156-519D-4053-BE29-EB57958C3E65} */
class DynModelModelFilter implements ModelFilter
{
	/** @modelguid {93AD481F-276B-4407-B7E2-B6ADCF3FBF2B} */
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return false;
	}


	/** @modelguid {B0379E2E-A72A-4CD0-A359-886079C3B4D6} */
	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return false;
	}
}

