package br.com.r4j.robosim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import javax.swing.JOptionPane;
import javax.swing.JSlider;
import javax.swing.event.EventListenerList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.commons.exception.ConcurrentException;
import br.com.r4j.commons.util.Collections;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EstimateConsumer;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.EstimatorEngineListener;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;

import com.vladium.utils.timing.*;


public abstract class RobotPlayer implements EstimatesEngine
{
	private static Log log = LogFactory.getLog(RobotPlayer.class.getName());
	private static Log logEstimates = LogFactory.getLog("estimates");

	private EventListenerList listenerList = null;
	
	protected int currentStep = -1;
	protected int currentProcessedStep = -1;

	protected long minWaitTime = 0;

	private ArrayList listEstRenderers = null;

	private ArrayList listSensors = null;
	private ArrayList listSensorModels = null;
	private ArrayList listEsts = null;
	private DynamicModel dynModel = null;

	private AbstractDoubleVector initialPosition = null;
	private AbstractDoubleSquareMatrix initialPositionCovar = null;

	private boolean bChangedEstimators = false;

	private AbstractDoubleVector [] realPoses = null;
	private long [] totalTimes = null;
	private AbstractDoubleVector [][] means = null;
	private AbstractDoubleSquareMatrix [][] covars = null;

	private boolean bgeneratingEstimates = false;
	
	private HashMap mapRendererInfos = null;

	private boolean bRunning = false;
	
	private static Boolean bLock = Boolean.TRUE; 

	private ITimer timer = null;

	private InfoSink infoSink = null;



	public RobotPlayer()
	{
		log.debug("RobotPlayer criado!!!: " + this.getClass().getName());
		listEstRenderers = new ArrayList(); 
		listenerList = new EventListenerList();
		listEsts = new ArrayList();
		listSensors = new ArrayList();
		listSensorModels = new ArrayList();
		mapRendererInfos = new HashMap();
		bChangedEstimators = true;
		bgeneratingEstimates = false;

		timer = TimerFactory.newTimer();
	}


	public InfoSink getInfoSink()	{return this.infoSink;}
	public void setInfoSink(InfoSink infoSink)	{this.infoSink = infoSink;}

	
	/**
	 * Usado para passar os dados das fontes internas para os sensores.
	 */
	protected abstract void gatherData(boolean bEngine);

	protected void logData(boolean bEngine)
	{
	}



	/**
	 * Retornar -1 indica que n�o sabe com antecend�ncia o n�mero de passos.
	 *
	 */
	public abstract int getNumberOfSteps();


	public abstract void sensorAdded(Sensor sns);


	public abstract void resetBuffers();


	public void setInitialState(AbstractDoubleVector initialPosition, AbstractDoubleSquareMatrix initialPositionCovar)
	{
		this.initialPosition = initialPosition;
		this.initialPositionCovar = initialPositionCovar;
	}


	public void addSensor(Sensor sns)
	{
		listSensors.add(sns);
		this.sensorAdded(sns);
	}


	public void addSensorModel(SensorModel snsMdl)	{listSensorModels.add(snsMdl);}


	public void addEstimator(Estimator est)
	{
		listEsts.add(est);
		bChangedEstimators = true;
	}


	public void setEstimatorInfo(Estimator est, EstimatorRendererInfo estInfo)
	{
		mapRendererInfos.put(est, estInfo);
	}


	public void setDynamicModel(DynamicModel dynModel) {this.dynModel = dynModel;}


	public List getSensors()	{return listSensors;}
	public List getSensorModels()	{return listSensorModels;}
	public List getEstimators()	{return listEsts;}
	public DynamicModel getDynamicModel() {return this.dynModel;}


	public void step() throws ConcurrentException
	{
		this.step(false);
	}


	protected void step(boolean bEngine) throws ConcurrentException
	{
//		log.debug("step:in");
		if (!bEngine && bgeneratingEstimates)
		{
			throw new ConcurrentException("Outra opera��o j� est� sendo realizada");
		}

		synchronized (bLock)
		{
//			log.debug("step:sync:in");
			if (bChangedEstimators)
			{
				this.prepareBuffers();
				bChangedEstimators = false;
			}
			
			if (bEngine)
			{
				this.doStep(bEngine);
				currentProcessedStep = currentStep;
			}
			else
			{
				if (currentStep == 0)
					this.fireBeginOfActions(new RobotPlayerEvent(this, currentStep));

				this.fireActionStarted(new RobotPlayerEvent(this, currentStep));

				if (currentProcessedStep >= currentStep)
					this.setupStep(currentStep);
				else if (currentStep >= this.getNumberOfSteps())
					log.debug("no more steps: " + currentStep);
				else
				{
					this.doStep(bEngine);
					currentProcessedStep = currentStep;
				}
			
				this.fireActionCompleted(new RobotPlayerEvent(this, currentStep));
				this.fireActionsUpdated(new RobotPlayerEvent(this, currentStep));

				int numOfSteps = this.getNumberOfSteps();
				if (numOfSteps != -1 && numOfSteps == currentStep + 1)
					this.fireEndOfActions(new RobotPlayerEvent(this, currentStep));
			}

			currentStep++;
//			log.debug("step:out");
		}
	}


	protected void setupStep(int idx)
	{
		currentStep = idx;

		if (currentStep == 0)
			this.fireBeginOfActions(new RobotPlayerEvent(this, currentStep));

		Iterator itEstRenderers = listEstRenderers.iterator();
		int counter = 0;
		while (itEstRenderers.hasNext())
		{
			EstimatorRenderer estRenderer = (EstimatorRenderer) itEstRenderers.next();
			estRenderer.setStep(currentStep);
		}

		this.fireActionCompleted(new RobotPlayerEvent(this, currentStep));
		this.fireActionsUpdated(new RobotPlayerEvent(this, currentStep));

		int numOfSteps = this.getNumberOfSteps();
		if (numOfSteps != -1 && numOfSteps == currentStep + 1)
			this.fireEndOfActions(new RobotPlayerEvent(this, currentStep));

		this.logData(false);
	}


	public void logStats()
	{
	}

	
	protected void doStep(boolean bEngine)
	{
		this.gatherData(bEngine);
		this.logData(bEngine);
		this.updateSensors(bEngine);
		this.updateSensorModels(bEngine);
		this.updateDynamicModel(bEngine);
		this.updateEstimators(bEngine);
		this.setInternalBuffers(bEngine);
		this.backpropagateEstimates(bEngine);
	}


	protected void updateSensors(boolean bEngine)
	{
//		log.debug("updateSensors");
		Iterator itSns = listSensors.iterator();
		while (itSns.hasNext())
		{
			Sensor sns = (Sensor) itSns.next();
//			log.debug("sns.getName() = " + sns.getName());
			sns.dataAvailable();
		}
	}


	protected void updateSensorModels(boolean bEngine)
	{
//		log.debug("updateSensorModels");
		Iterator itSnsMdls = listSensorModels.iterator();
		while (itSnsMdls.hasNext())
		{
			SensorModel sns = (SensorModel) itSnsMdls.next();
			sns.dataAvailable();
		}
	}


	protected void updateDynamicModel(boolean bEngine)
	{
		dynModel.dataAvailable();
	}


	protected void updateEstimators(boolean bEngine)
	{
		for (int i = 0; i < listEsts.size(); i++)
		{
			Estimator est = (Estimator) listEsts.get(i);
			timer.reset();
			timer.start();
			est.estimate(!bEngine);
			timer.stop();
			totalTimes[i] += (long) (1000*timer.getDuration());
		}
	}


	protected void prepareBuffers()
	{
		totalTimes = new long[listEsts.size()];
		if (this.getNumberOfSteps() != -1)
		{
			realPoses = new AbstractDoubleVector[this.getNumberOfSteps()];
			means = new AbstractDoubleVector[listEsts.size()][this.getNumberOfSteps()];
			covars = new AbstractDoubleSquareMatrix[listEsts.size()][this.getNumberOfSteps()];
		}
		else
		{
			realPoses = new AbstractDoubleVector[500];
			means = new AbstractDoubleVector[listEsts.size()][500];
			covars = new AbstractDoubleSquareMatrix[listEsts.size()][500];
		}
		currentProcessedStep = -1;
		currentStep = 0;
		this.resetBuffers();
	}


	protected void growBuffers()
	{
		int sizeNew = covars[0].length*2;
		AbstractDoubleVector [] realPosesNew = new AbstractDoubleVector[sizeNew];
		AbstractDoubleVector [][] meansNew = new AbstractDoubleVector[listEsts.size()][sizeNew];
		AbstractDoubleSquareMatrix [][] covarsNew = new AbstractDoubleSquareMatrix[listEsts.size()][sizeNew];
		for (int j = 0; j < covars.length; j++)
		{
			for (int i = 0; i < covars[0].length; i++)
			{
				meansNew[j][i] = means[j][i];
				covarsNew[j][i] = covars[j][i];
			}
		}
		if (realPoses != null && realPoses.length > 0) for (int i = 0; i < realPoses.length; i++)
		{
			realPosesNew[i] = realPoses[i]; 
		}
		means = meansNew;
		covars = covarsNew;
		realPoses = realPosesNew;
	}


	protected void setInternalBuffers(boolean bEngine)
	{
		Iterator itEsts = listEsts.iterator();
		int estIdx = 0;
		logEstimates.debug("----------- it: " + currentStep);
		while (itEsts.hasNext())
		{
			Estimator est = (Estimator) itEsts.next();
			AbstractDoubleVector mean = est.getMean();
			AbstractDoubleSquareMatrix covar = est.getCovariance();

			logEstimates.debug("name: " + est.getName());
			logEstimates.debug("\t" + MatrixUtil.toString(mean, 9, 2));
			logEstimates.debug("\r\n" + MatrixUtil.toString(covar, 9, 4));

			means[estIdx][currentStep] = mean;
			covars[estIdx][currentStep] = covar;

			estIdx++;
		}
		realPoses[currentStep] = this.getActualRealPose();
		
		if ((bEngine && this.getNumberOfSteps() == -1 && currentStep == covars[0].length - 1) || (!bEngine && currentStep == covars[0].length - 1))
		{
			this.growBuffers();
		}
	}

	
	protected abstract AbstractDoubleVector getActualRealPose();
	

	protected void backpropagateEstimates(boolean bEngine)
	{
		Iterator itSnsMdls = listSensorModels.iterator();
		while (itSnsMdls.hasNext())
		{
			Object sns = itSnsMdls.next();
			if (sns instanceof EstimateConsumer)
				((EstimateConsumer) sns).estimateAvailable(means[0][currentStep], covars[0][currentStep]);
		}
		if (dynModel instanceof EstimateConsumer)
			((EstimateConsumer) dynModel).estimateAvailable(means[0][currentStep], covars[0][currentStep]);
	}


	public void seek(int idx)
	{
		if (currentProcessedStep >= idx)
		{
			this.setupStep(idx);
		}
		else if (idx >= this.getNumberOfSteps())
		{
			log.debug("passo n�o existe: " + idx);
		}
		else
		{
			while (idx > currentProcessedStep)
			{
				this.step();
			}
		}
	}


	public void pause()
	{
		bRunning = false;
	}


	public void play(JSlider slider)
	{
		try
		{
			if (!bRunning)
			{
				Thread threadPlay = new Thread(new Player(slider));
				threadPlay.setDaemon(true);
				threadPlay.setName("Action Player");
				threadPlay.start(); 
				bRunning = true;
			}
			else
			{
				JOptionPane.showMessageDialog(null, 
						"O player j� est� acionado!",
						"O player j� est� acionado!",
						JOptionPane.ERROR_MESSAGE);
			}
		}
		catch (Exception e)
		{
			JOptionPane.showMessageDialog(null, 
					"Ocorreu um erro com o player",
					"Ocorreu um erro com o player",
					JOptionPane.ERROR_MESSAGE);
			log.error("erro!", e);
		}
	}


	class Player implements Runnable
	{
		private JSlider slider = null;
		

		public Player(JSlider slider)
		{
			this.slider = slider;
		}
		

		public void run()
		{
			slider.setEnabled(false);
			try
			{
				while (currentStep < getNumberOfSteps() && bRunning)
				{
					long timeBefore = System.currentTimeMillis();
//					slider.getModel().setValue(slider.getModel().getValue() + 1);
					step();
					long timeAfter = System.currentTimeMillis();
					if (timeAfter - timeBefore < minWaitTime)
						Thread.sleep(minWaitTime - (timeAfter - timeBefore));
				}
				logStats();
			}
			catch (Exception e)
			{
				log.error("erro!", e);
			}
			bRunning = false;
			slider.setEnabled(true);
		}
	}


	public void setPlayRate(long minWaitTime)
	{
		this.minWaitTime = minWaitTime;
	}


	public int getCurrentStep()	{return currentStep;}


	public void addRobotPlayerListener(RobotPlayerListener listener)
		{listenerList.add(RobotPlayerListener.class, listener);}
	public void removeRobotPlayerListener(RobotPlayerListener listener)
		{listenerList.remove(RobotPlayerListener.class, listener);}


	protected void fireActionStarted(RobotPlayerEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == RobotPlayerListener.class)
				((RobotPlayerListener) listeners[i+1]).actionStarted(e);
	}


	protected void fireActionCompleted(RobotPlayerEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == RobotPlayerListener.class)
				((RobotPlayerListener) listeners[i+1]).actionCompleted(e);
	}


	protected void fireEndOfActions(RobotPlayerEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == RobotPlayerListener.class)
				((RobotPlayerListener) listeners[i+1]).endOfActions(e);
	}


	protected void fireBeginOfActions(RobotPlayerEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == RobotPlayerListener.class)
				((RobotPlayerListener) listeners[i+1]).beginOfActions(e);
	}


	protected void fireActionsUpdated(RobotPlayerEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == RobotPlayerListener.class)
				((RobotPlayerListener) listeners[i+1]).actionsUpdated(e);
	}


	public void processData(int numberOfTries, NoiseControlInfo noiseInfo, EstimatorEngineListener estList)
	{
		synchronized (this)
		{
			NoiseControlInfo [] arrayNoiseInfoOld = new NoiseControlInfo[listSensors.size()]; 
			bgeneratingEstimates = true;
			AbstractDoubleVector [][] meansTmp = means;
			AbstractDoubleSquareMatrix [][] covarsTmp = covars;
			int currentProcessedStepTmp = currentProcessedStep;
			int currentStepTmp = currentStep;

			int numSteps = this.getNumberOfSteps();
			if (numSteps == -1)
				numSteps = currentProcessedStep + 1;

			Iterator itSns = listSensors.iterator(); int i = 0;
			while (itSns.hasNext())
			{
				Sensor sns = (Sensor) itSns.next();
				if (sns instanceof NoiseControl)
				{
					arrayNoiseInfoOld[i] = ((NoiseControl) sns).getNoiseControlInfo();
					((NoiseControl) sns).setNoiseControlInfo(noiseInfo);
				}
				i++;
			}

			totalTimes = new long[listEsts.size()];
			for (int idxRun = 0; idxRun < numberOfTries; idxRun++)
			{
				this.resetPlayer();
				for (int idxStep = 0; idxStep < numSteps; idxStep++)
					this.step(true);
					
				estList.estimationIterationPerformed(realPoses, means, covars, idxRun, listEsts.size(), numSteps);
			}

			for (int iBas = 0; iBas < listEsts.size(); iBas++)
				totalTimes[iBas] /= (numberOfTries*numSteps);

			this.resetPlayer();

			itSns = listSensors.iterator(); i = 0;
			while (itSns.hasNext())
			{
				Sensor sns = (Sensor) itSns.next();
				if (sns instanceof NoiseControl)
					((NoiseControl) sns).setNoiseControlInfo(arrayNoiseInfoOld[i]);
				i++;
			}
			
			if (currentStepTmp == -1)
				this.resetPlayer();
			else
			{
				means = meansTmp;
				covars = covarsTmp;
				currentProcessedStep = currentProcessedStepTmp;
				currentStep = currentStepTmp;
			}
			bgeneratingEstimates = false;
		}
	}


	public AbstractDoubleVector[] getRealValues()
	{
		return realPoses;
	}


	public long [] getTotalTimes()
	{
		return totalTimes;
	}


	public List getEstimatorRendererInfos()
	{
		return Collections.createListFromValues(mapRendererInfos);
	}


	public void resetPlayer()
	{
		bChangedEstimators = true;
		currentStep = 0;
		currentProcessedStep = -1;

		Iterator itEsts = listEsts.iterator();
		while (itEsts.hasNext())
		{
			Estimator est = (Estimator) itEsts.next();
			est.setState(initialPosition, initialPositionCovar);
		}

		Iterator itSns = listSensors.iterator();
		while (itSns.hasNext())
		{
			Sensor sns = (Sensor) itSns.next();
			sns.reset();
		}
	}


	public void addEstimatorRenderer(EstimatorRenderer estPoseRenderer)
	{
		listEstRenderers.add(estPoseRenderer);
	}
}

/*

3537

select * from odf_ca_project where ecb_co = '131292'
select * from srm_projects where unique_name = '131292'

*/
