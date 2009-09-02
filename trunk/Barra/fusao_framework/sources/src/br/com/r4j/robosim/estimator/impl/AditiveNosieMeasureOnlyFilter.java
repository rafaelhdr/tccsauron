package br.com.r4j.robosim.estimator.impl;

import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.Collections;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;


public class AditiveNosieMeasureOnlyFilter extends BaseEstimator
{
	private static Log log = LogFactory.getLog(AditiveNosieMeasureOnlyFilter.class.getName());
	private static Log logTime = LogFactory.getLog("time");
	
	private Sensor sens = null;
	private SensorModel sensModel = null;
	private InvertibleEKFDoubleVectorFunction invEkfFunc = null;


	public AditiveNosieMeasureOnlyFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Observação";
	}


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		if (!(sensModel instanceof InvertibleEKFDoubleVectorFunction))
			throw new ConfiguratorException("Sensor Model " + sensModel.getName() + " não implementa InvertibleEKFDoubleVectorFunction");

		this.sens = sens;
		this.sensModel = sensModel;
		this.invEkfFunc = (InvertibleEKFDoubleVectorFunction) sensModel;

		super.addSensorModel(sensModel, sens);
	}

	
	public List getSensorModels()
	{
		return Collections.createList(sensModel);
	}

	
	protected void doEstimate()
	{
		boolean bResults = sens.hasNewData(sensModel, this);
		if (bResults)
		{
			AbstractDoubleVector vectReadings = new DoubleVector(sens.getDataDimension(sensModel));
			sens.getData(vectReadings, sensModel);
			AbstractDoubleSquareMatrix snsR = sens.getDataCovariance(sensModel);

			stateEstimate = invEkfFunc.produceInverseResults(stateEstimate, vectReadings);
	
			AbstractDoubleMatrix invHi = invEkfFunc.getInverseTransitionMatrixJacobian(vectReadings);
			AbstractDoubleSquareMatrix R = sensModel.getObservationCovariance(snsR);
			AbstractDoubleMatrix reTmp = invHi.multiply(R.multiply((AbstractDoubleMatrix) invHi.transpose()));
			stateCovarEstimate = MatrixUtil.convert2SquareMatrix(reTmp);
			
			this.setState(stateEstimate, stateCovarEstimate);
		}
	}


	public static ModelFilter getModelFilter()
	{
		return new InvertibleEKFModelFilter();
	}
}


class InvertibleEKFModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return (dynModel instanceof InvertibleEKFDoubleVectorFunction);
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return (sensModel instanceof InvertibleEKFDoubleVectorFunction);
	}
}
