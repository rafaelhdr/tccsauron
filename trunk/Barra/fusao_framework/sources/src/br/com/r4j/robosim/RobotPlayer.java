package br.com.r4j.robosim;

import java.util.ArrayList;
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
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.math.VectorScrambler;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EstimateConsumer;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;


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
	private AbstractDoubleVector [][] means = null;
	private AbstractDoubleSquareMatrix [][] covars = null;

	private boolean bgeneratingEstimates = false;
	private AbstractDoubleVector [][][] estimatedMeans = null;
	private AbstractDoubleSquareMatrix [][][] estimatedCovars = null;

	private boolean bRunning = false;
	
	private static Boolean bLock = Boolean.TRUE; 


	public RobotPlayer()
	{
		log.debug("RobotPlayer criado!!!: " + this.getClass().getName());
		listEstRenderers = new ArrayList(); 
		listenerList = new EventListenerList();
		listEsts = new ArrayList();
		listSensors = new ArrayList();
		listSensorModels = new ArrayList();
		bChangedEstimators = true;
		bgeneratingEstimates = false;
	}

	
	/**
	 * Usado para passar os dados das fontes internas para os sensores.
	 */
	protected abstract void gatherData();


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
		log.debug("step:in");
		if (!bEngine && bgeneratingEstimates)
		{
			throw new ConcurrentException("Outra opera��o j� est� sendo realizada");
		}

		synchronized (bLock)
		{
			log.debug("step:sync:in");
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
				{
					this.setupStep(currentStep);
				}
				else if (currentStep >= this.getNumberOfSteps())
				{
					log.debug("no more steps: " + currentStep);
				}
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
			log.debug("step:out");
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
	}


	protected void doStep(boolean bEngine)
	{
		this.gatherData();
		this.updateSensors();
		this.updateSensorModels();
		this.updateDynamicModel();
		this.updateEstimators(bEngine);
		this.setInternalBuffers();
		this.backpropagateEstimates();
	}


	protected void updateSensors()
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


	protected void updateSensorModels()
	{
//		log.debug("updateSensorModels");
		Iterator itSnsMdls = listSensorModels.iterator();
		while (itSnsMdls.hasNext())
		{
			SensorModel sns = (SensorModel) itSnsMdls.next();
			sns.dataAvailable();
		}
	}


	protected void updateDynamicModel()
	{
		dynModel.dataAvailable();
	}


	protected void updateEstimators(boolean bEngine)
	{
		Iterator itEsts = listEsts.iterator();
		while (itEsts.hasNext())
		{
			Estimator est = (Estimator) itEsts.next();
			est.estimate(!bEngine);
		}
	}


	protected void prepareBuffers()
	{
		if (this.getNumberOfSteps() != -1)
		{
			means = new AbstractDoubleVector[listEsts.size()][this.getNumberOfSteps()];
			covars = new AbstractDoubleSquareMatrix[listEsts.size()][this.getNumberOfSteps()];
		}
		else
		{
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
		means = meansNew;
		covars = covarsNew;
	}


	protected void setInternalBuffers()
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
		if (currentStep == covars[0].length - 1)
		{
			this.growBuffers();
		}
	}


	protected void backpropagateEstimates()
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


	public void processData(int numberOfTries, NoiseControlInfo noiseInfo)
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

			estimatedMeans = new AbstractDoubleVector[numberOfTries][][];
			estimatedCovars = new AbstractDoubleSquareMatrix[numberOfTries][][];
			for (int idxRun = 0; idxRun < numberOfTries; idxRun++)
			{
				this.resetPlayer();
				for (int idxStep = 0; idxStep < numSteps; idxStep++)
					this.step(true);

				estimatedMeans[idxRun] = means;
				estimatedCovars[idxRun] = covars;
			}
			this.resetPlayer();

			itSns = listSensors.iterator(); i = 0;
			while (itSns.hasNext())
			{
				Sensor sns = (Sensor) itSns.next();
				if (sns instanceof NoiseControl)
					((NoiseControl) sns).setNoiseControlInfo(arrayNoiseInfoOld[i]);
				i++;
			}

			means = meansTmp;
			covars = covarsTmp;
			currentProcessedStep = currentProcessedStepTmp;
			currentStep = currentStepTmp;
			bgeneratingEstimates = false;
		}
	}


	public AbstractDoubleVector [][][] getEstimatesMean()
	{
		return estimatedMeans;
	}


	public AbstractDoubleSquareMatrix [][][] getEstimatesCovariance()
	{
		return estimatedCovars;
	}


	/**
	 * 
	 */
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
	}


	public void addEstimatorRenderer(EstimatorRenderer estPoseRenderer)
	{
		listEstRenderers.add(estPoseRenderer);
	}
}

