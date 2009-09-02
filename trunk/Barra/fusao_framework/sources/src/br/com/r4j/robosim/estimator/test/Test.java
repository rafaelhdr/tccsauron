package br.com.r4j.robosim.estimator.test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.configurator.WebStartConfigurator;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.impl.AditiveNosieExtendedKalmanFilter;
import br.com.r4j.robosim.estimator.impl.AditiveNosieSquareRootUnscentedKalmanFilter;
import br.com.r4j.robosim.estimator.impl.AditiveNosieUnscentedKalmanFilter;


public class Test
{
	private static Log log = LogFactory.getLog(Test.class.getName());
	private static Log logOutput = LogFactory.getLog("output");

	
	private List listEstimators = null;
	private List listModels = null;
	private List listDynModels = null;

	private HSensor hSns = null;
	private FSensor fSns = null;

	
	public static void main(String [] args)
	{
		try
		{
			WebStartConfigurator.createConfigurator("conf/conf.xml");
			Configurator conf = Configurator.getInstance();
			PropertiesHolder props = conf.getPropertiesHolder();

			Test app = new Test();
			app.createComponents();
			app.configureComponents(props);
			app.linkComponents();
			app.execute();
		}
		catch (ConfiguratorException e)
		{
			e.printStackTrace();
			return;
		}
		catch (Exception e)
		{
			e.printStackTrace();
			return;
		}
	}



	public Test()
	{
	}


	private void createComponents()
	{
		listEstimators = new ArrayList();
		listModels = new ArrayList();
		listDynModels = new ArrayList();

		listEstimators.add(new AditiveNosieExtendedKalmanFilter());
		listEstimators.add(new AditiveNosieUnscentedKalmanFilter());
		listEstimators.add(new AditiveNosieSquareRootUnscentedKalmanFilter());
//		listEstimators.add(new AditiveNosieEigenvector2SigmaPointsUnscentedKalmanFilter());

		listModels.add(new HModel());
		listModels.add(new HModel());
		listModels.add(new HModel());
//		listModels.add(new HModel());

		listDynModels.add(new FModel());
		listDynModels.add(new FModel());
		listDynModels.add(new FModel());
//		listDynModels.add(new FModel());

		hSns = new HSensor();
		fSns = new FSensor();
	}


	private void configureComponents(PropertiesHolder props)
	{
		hSns.configure(props, "/hsensor");
		fSns.configure(props, "/fsensor");

		try
		{
			hSns.loadData();
			fSns.loadData();
		}
		catch (IOException e)
		{
			log.error("erro", e);
		}
	}


	private void linkComponents() throws ConfiguratorException
	{
		Iterator itEsts = listEstimators.iterator();
		Iterator itModels = listModels.iterator();
		Iterator itDynModels = listDynModels.iterator();
		while (itEsts.hasNext())
		{
			Estimator est = (Estimator) itEsts.next();
			SensorModel snsModel = (SensorModel) itModels.next();
			DynamicModel dynModel = (DynamicModel) itDynModels.next();

			dynModel.setSensor(fSns);
			snsModel.setSensor(hSns);

			est.setDynamicModel(dynModel, fSns);
			est.addSensorModel(snsModel, hSns);
			
			est.setState(new DoubleVector(new double [] {0, 0}),
						new DoubleSquareMatrix(new double [][] {{1, 0}, {0, 1}}));
		}
	}


	public void execute()
	{

		Iterator itEsts = listEstimators.iterator();
		Iterator itModels = listModels.iterator();
		Iterator itDynModels = listDynModels.iterator();
		while (itEsts.hasNext())
		{
			Estimator est = (Estimator) itEsts.next();
			logOutput.debug(est.getName() + ":");

			SensorModel snsModel = (SensorModel) itModels.next();
			DynamicModel dynModel = (DynamicModel) itDynModels.next();

			hSns.resetData();
			fSns.resetData();

			int N = hSns.getReadingsCount();
			for (int i = 0; i < N; i++)
			{
				hSns.dataAvailable();
				fSns.dataAvailable();

				snsModel.dataAvailable();
				dynModel.dataAvailable();

				est.estimate(false);

				AbstractDoubleVector mean = est.getMean();
				logOutput.debug(MatrixUtil.toString(mean, 10, 1, 6, 6, "\t"));
			}
		}
	}
}

