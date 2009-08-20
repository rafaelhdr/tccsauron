package br.com.r4j.robosim.model;

import java.util.Iterator;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.robosim.BaseSonarSensor;
import br.com.r4j.robosim.MapDependent;
import br.com.r4j.robosim.Wall;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction;
import br.com.r4j.robosim.estimator.PredictedEstimateConsumer;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


/**
 * 
 * @author giord
 *
 * Condições para produzir resultado:
 * 
 * 1 - O sonar certo ter produzido uma leitura.
 * 2 - Movimento pode ser considerado retilíneo.
 * 3 - Estar rastreando com sucesso uma parede.
 * 
 * 
 * Condições para determinar se uma parede esta sendo rastrada.
 * 1 - O sonar certo ter produzido uma leitura.
 * 2 - Movimento pode ser considerado retilíneo.
 * 
 * 
 * Operação
 * 
 * 1 - Determinar se uma parede está sendo observada ou não.
 * 2 - Verifica se uma parede está sendo rastreada.
 * 3 - Se não, procura uma para rastrear.
 * 4 - Se sim, verifica se a mesma está sendo rastreada ainda.
 * 5 - Se for a mesma, marca como sucesso, e zera contador de falhas.
 * 6 - Se não, marca como insucesso e incrementa contador de falhas.
 * 7 - Se contadorde falhas etiver zerado e mais de uma associação com sucesso tiver sido feita, então libera observação.
 * 8 - Se contador de falhas estiver em 3, então resseta tudo, limpando o buffer de parede associada.
 * 
 */
//, InvertibleEKFDoubleVectorFunction
public class SonarModel implements SensorModel, Configurable, MapDependent, PredictedEstimateConsumer, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, PosteriorProbabilityDensityFunction
{
	private static Log log = LogFactory.getLog(SonarModel.class.getName());
	private static Log logModel = LogFactory.getLog("sonarmodel");

	private WorldMap map = null;
	
	private double phoErrorFront4mm = 0.05;
	private int idxSonar = 0;
	
	private BaseSonarSensor sns = null;

	private int currentIdx = 0;
	private double [] arrayReadings = null; 
	private double [][] arrayPoses = null; 
	private boolean [] arrayValid = null; 

	private double wallRejectionValue = 1.645;
	private double wallRejectionValue2 = wallRejectionValue*wallRejectionValue;
	
	private Wall [] arrayAssociatedWalls = null;
	private int [] arrayFailCounter = null;
	private int [] arrayHitCounter = null;
	private int [] arrayMissCounter = null;
	
	private AbstractDoubleSquareMatrix obsCovariance = null; 

	private int uniqueId = 0;
	private static int uniqueIdCount = 0;
	
	
	public SonarModel()
	{
		uniqueId = uniqueIdCount++;
		log.debug("SonarModel: " + uniqueId);

		currentIdx = 0;
		arrayReadings = new double[3]; 
		arrayPoses = new double[3][3]; 
		arrayValid = new boolean[3];  
		arrayAssociatedWalls = new Wall[1];
		arrayFailCounter = new int[1];
		arrayHitCounter = new int[1];
		arrayMissCounter = new int[1];
	}
	

	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/idx_sonar"))
			idxSonar = props.getIntegerProperty(strBaseKey + "/idx_sonar").intValue();
		if (props.containsProperty(strBaseKey + "/phoErrorFront4mm"))
			phoErrorFront4mm = props.getDoubleProperty(strBaseKey + "/phoErrorFront4mm").doubleValue();

		log.debug("idxSonar: " + idxSonar + ", phoErrorFront4mm: " + phoErrorFront4mm);
	}


	public void setSensor(Sensor sns)
	{
		this.sns = (BaseSonarSensor) sns;
	}


	public String getName()
	{
		return "Sonar " + idxSonar + " Model ";
	}

	
	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
	}


	public void predictedEstimateAvailable(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar)
	{
		int currentIdxTmp = (currentIdx + 1)%arrayReadings.length;
		arrayPoses[currentIdxTmp][0] = mean.getComponent(0); 
		arrayPoses[currentIdxTmp][1] = mean.getComponent(1); 
		arrayPoses[currentIdxTmp][2] = mean.getComponent(2);
	}


	public int getDataDimension()
	{
		return 1;
	}


	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		double reading = vectReadings.getComponent(idxSonar);
		logModel.debug(uniqueId + ":testing reading: " + reading + ", reading != -1: " + (reading != -1) + ", idxSonar = " + idxSonar);
		log.debug(uniqueId + ":testing reading: " + reading + ", reading != -1: " + (reading != -1));
		if (reading == -1)
			return false;
			
		currentIdx = (currentIdx + 1)%arrayReadings.length;
		arrayReadings[currentIdx] = reading;
		if (sns.isOutOfRange(reading))
		{
			arrayValid[currentIdx] = false;
			for (int i = 0; i < arrayAssociatedWalls.length; i++)
			{
				arrayFailCounter[i]++;
				arrayMissCounter[i]++;
			}
		}
		else
			arrayValid[currentIdx] = true;

		logModel.debug(uniqueId + ":" + arrayValid[0] + ":" + arrayValid[1] + ":" + arrayValid[2]);
		boolean bCanProduceResults = false;
		if (arrayValid[0] && arrayValid[1] && arrayValid[2])
		{
			bCanProduceResults = this.testForWall();
			logModel.debug(uniqueId + ":this.testForWall(): " + bCanProduceResults);
			
			if (!bCanProduceResults)
			{
				for (int i = 0; i < arrayAssociatedWalls.length; i++)
				{
					arrayFailCounter[i]++;
					arrayMissCounter[i]++;
				}
			}
		}

		if (obsCovariance == null)
			obsCovariance = new DoubleSquareMatrix(1);
		obsCovariance.setElement(0, 0, sensorCovariance.getElement(idxSonar, idxSonar));
		return bCanProduceResults;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 * 2 - Verifica se uma parede está sendo rastreada.
	 * 3 - Se não, procura uma para rastrear.
	 * 4 - Se sim, verifica se a mesma está sendo rastreada ainda.
	 * 5 - Se for a mesma, marca como sucesso, e zera contador de falhas.
	 * 6 - Se não, marca como insucesso e incrementa contador de falhas.
	 * 7 - Se contador de falhas etiver zerado e mais de uma associação com sucesso tiver sido feita, então libera observação.
	 * 8 - Se contador de falhas estiver em 3, então resseta tudo, limpando o buffer de parede associada.
	 */
	public void produceResults(AbstractDoubleVector state, AbstractDoubleVector obsPredicted, AbstractDoubleSquareMatrix stateCovar)
	{
		this.processProduceResults(state, stateCovar.getElement(0, 0), stateCovar.getElement(1, 1), stateCovar.getElement(2, 2), 0);
		logModel.debug(uniqueId + ":arrayMissCounter[0]:" + arrayMissCounter[0]);
		if (arrayMissCounter[0] == 0)
		{
			double predObs = this.predictObservation(state.getComponent(0), state.getComponent(1), state.getComponent(2), arrayAssociatedWalls[0]);
			logModel.debug(uniqueId + ":predicted:" + predObs);
			obsPredicted.setComponent(0, predObs);
		}
		else if (arrayMissCounter[0] > 2)
		{
			obsPredicted.setComponent(0, arrayReadings[currentIdx]);
			obsCovariance.setElement(0, 0, (1000*1000*1000)*(1000*1000));
			arrayAssociatedWalls[0] = null;
		}
		else
		{
			obsPredicted.setComponent(0, arrayReadings[currentIdx]);
			obsCovariance.setElement(0, 0, (1000*1000*1000)*(1000*1000));
		}
	}


	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance)
	{
		return MatrixUtil.clone(obsCovariance);
	}


	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings)
	{
		double reading = sensorReadings.getComponent(idxSonar);
		AbstractDoubleVector obs = new DoubleVector(1);
		obs.setComponent(0, reading);
		return obs;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface EKFDoubleVectorFunction

	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		AbstractDoubleMatrix H = new DoubleMatrix(1, 3);
		Wall wall = arrayAssociatedWalls[0];
		if (wall != null)
		{
			if (FunctionsR.angularDist(state.getComponent(2) + sns.getThetaS(idxSonar), wall.getTheta()) > Math.PI/2)
			{
				H.setElement(0, 0, +wall.getCosTheta());
				H.setElement(0, 1, +wall.getSinTheta());
		
				double thetaMix = (state.getComponent(2) - wall.getTheta());
				double cosMix = Math.cos(thetaMix);
				double sinMix = Math.sin(thetaMix);
				H.setElement(0, 2, -1*(sns.getXS(idxSonar)*sinMix + sns.getYS(idxSonar)*cosMix));
			}
			else
			{
				H.setElement(0, 0, -wall.getCosTheta());
				H.setElement(0, 1, -wall.getSinTheta());
		
//				double thetaMix = thetaS + state.getComponent(2) - wall.getTheta();
				double thetaMix = state.getComponent(2) - wall.getTheta();
				double cosMix = Math.cos(thetaMix);
				double sinMix = Math.sin(thetaMix);
				H.setElement(0, 2, sns.getXS(idxSonar)*sinMix + sns.getYS(idxSonar)*cosMix);
			}
		}
		else
		{
			H.setElement(0, 0, 1);
			H.setElement(0, 1, 1);
			H.setElement(0, 2, 1);
		}
		return H;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	public void produceResults(AbstractDoubleMatrix stateSigma, AbstractDoubleVector state, AbstractDoubleMatrix sigmaError, AbstractDoubleMatrix obsPredictedSigma, AbstractDoubleSquareMatrix stateCovar)
	{
		this.processProduceResults(state, stateCovar.getElement(0, 0), stateCovar.getElement(1, 1), stateCovar.getElement(2, 2), 0);
		logModel.debug(uniqueId + ":produceResults[" + arrayMissCounter[0]);
		if (arrayMissCounter[0] == 0)
		{
			for (int idxInput = 0; idxInput < stateSigma.columns(); idxInput++)
			{
				double predObs = this.predictObservation(stateSigma.getElement(0, idxInput), 
														 stateSigma.getElement(1, idxInput), 
														 stateSigma.getElement(2, idxInput), 
														 arrayAssociatedWalls[0]) + sigmaError.getElement(0, idxInput);
				logModel.debug(uniqueId + ":predicted[" + idxInput + "]:" + predObs);
				obsPredictedSigma.setElement(0, idxInput, predObs);
			}
		}
		else if (arrayMissCounter[0] > 2)
		{
			for (int idxInput = 0; idxInput < stateSigma.columns(); idxInput++)
				obsPredictedSigma.setElement(0, idxInput, arrayReadings[currentIdx]);
			obsCovariance.setElement(0, 0, (1000*1000*1000)*(1000*1000));
			arrayAssociatedWalls[0] = null;
		}
		else
		{
			for (int idxInput = 0; idxInput < stateSigma.columns(); idxInput++)
				obsPredictedSigma.setElement(0, idxInput, arrayReadings[currentIdx]);
			obsCovariance.setElement(0, 0, (1000*1000*1000)*(1000*1000));
		}
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface PosteriorProbabilityDensityFunction

	private AbstractDoubleSquareMatrix covarInv = null;
	private double reading = 0;
	private double pdfConst = 0;

	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar)
	{
		this.obsCovariance = covar;
		covarInv = obsCovariance.inverse();
		logModel.debug(uniqueId + ":obsCovariance: \r\n" + MatrixUtil.toString(obsCovariance, 9, 4));
		logModel.debug(uniqueId + ":covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
		pdfConst = 1.0/Math.sqrt(((AbstractDoubleSquareMatrix) obsCovariance.scalarMultiply(2*Math.PI)).det());
		logModel.debug(uniqueId + ":pdfConst: " + pdfConst);
		if (pdfConst < 1)
			pdfConst = 1;
		logModel.debug(uniqueId + ":pdfConst(2): " + pdfConst);
	}


	public void copyParticleData(int idxFrom, int idxTo)
	{
		if (arrayAssociatedWalls.length <= idxFrom)
			this.changeArrayWallSize(idxFrom + 1);

		if (arrayAssociatedWalls.length <= idxTo)
			this.changeArrayWallSize(idxTo + 1);

		arrayAssociatedWalls[idxTo] = arrayAssociatedWalls[idxFrom];
		arrayFailCounter[idxFrom] = arrayFailCounter[idxFrom];
		arrayHitCounter[idxFrom] = arrayHitCounter[idxFrom];
		arrayMissCounter[idxFrom] = arrayMissCounter[idxFrom];
	}


	public void setObservations(AbstractDoubleVector readings)
	{
		this.reading = readings.getComponent(0);
	}

	
	public double stateProbability(AbstractDoubleMatrix inputStates, int rowObjectCount)
	{
		logModel.debug(uniqueId + ":stateProbability:" +  rowObjectCount);
		if (arrayAssociatedWalls.length >= rowObjectCount)
			this.changeArrayWallSize(rowObjectCount + 1);

		AbstractDoubleVector state = MatrixUtil.getRow(inputStates, rowObjectCount);
		this.processProduceResults(state, -1, -1, -1, rowObjectCount);
		
		if (arrayMissCounter[rowObjectCount] == 0)
		{
			double predObs = this.predictObservation(state.getComponent(0), state.getComponent(1), state.getComponent(2), arrayAssociatedWalls[rowObjectCount]);
			
			double diff = predObs - reading;
			double expoent = diff*covarInv.getElement(0, 0)*diff;
			return pdfConst*Math.exp(-1.0/2.0*expoent);
		}
		else
		{
			logModel.warn(uniqueId + ":stateProbability: retornando 0 (zero !!)!!");
			return 0;
		}
	}	


//	////////////////////////////////////////////////////////////////////////////////
//	/// Método internos

	
	private void changeArrayWallSize(int sizeNew)
	{
		Wall [] arrayAssociatedWallsTmp = new Wall[sizeNew];
		int [] arrayFailCounterTmp = new int[sizeNew];
		int [] arrayHitCounterTmp = new int[sizeNew];
		int [] arrayMissCounterTmp = new int[sizeNew];
		for (int i = 0; i < sizeNew && i < arrayAssociatedWalls.length; i++)
		{
			arrayAssociatedWallsTmp[i] = arrayAssociatedWalls[i];
			arrayFailCounterTmp[i] = arrayFailCounter[i];
			arrayHitCounterTmp[i] = arrayHitCounter[i];
			arrayMissCounterTmp[i] = arrayMissCounter[i];
		}
		for (int i = arrayAssociatedWalls.length; i < sizeNew; i++)
		{
			arrayFailCounterTmp[i] = 0;
			arrayHitCounterTmp[i] = 0;
			arrayMissCounterTmp[i] = 0;
		}
		arrayAssociatedWalls = arrayAssociatedWallsTmp;
		arrayFailCounter = arrayFailCounterTmp;
		arrayHitCounter = arrayHitCounterTmp;
		arrayMissCounter = arrayMissCounterTmp;
	}


	private boolean testForWall()
	{
		int idx1 = (arrayReadings.length + currentIdx - 2)%arrayReadings.length;
		int idx2 = (arrayReadings.length + currentIdx - 1)%arrayReadings.length;
		int idx3 = (arrayReadings.length + currentIdx - 0)%arrayReadings.length;

		double dTheta1 = arrayPoses[idx1][2];
		double dTheta2 = arrayPoses[idx2][2];
		double dTheta3 = arrayPoses[idx3][2];

		if (FunctionsR.angularDist(dTheta1, dTheta2) > Math.PI/6 || 
		    FunctionsR.angularDist(dTheta2, dTheta3) > Math.PI/6)
	    {
			logModel.debug(uniqueId + ":FunctionsR.angularDist(dTheta1, dTheta2) = " + FunctionsR.angularDist(dTheta1, dTheta2));
			logModel.debug(uniqueId + ":FunctionsR.angularDist(dTheta2, dTheta3) = " + FunctionsR.angularDist(dTheta2, dTheta3));
			
			return false;
	    }
		
		double dX12 = arrayPoses[idx2][0] - arrayPoses[idx1][0];
		double dY12 = arrayPoses[idx2][1] - arrayPoses[idx1][1];
		double d12 = Math.sqrt(dX12*dX12 + dY12*dY12);

		double dX23 = arrayPoses[idx3][0] - arrayPoses[idx2][0];
		double dY23 = arrayPoses[idx3][1] - arrayPoses[idx2][1];
		double d23 = Math.sqrt(dX23*dX23 + dY23*dY23);
		
		double r12 = arrayReadings[idx1] - arrayReadings[idx2]; 
		double r23 = arrayReadings[idx2] - arrayReadings[idx3];

		logModel.debug(uniqueId + ":arrayReadings[idx1] = " + arrayReadings[idx1] + ", arrayReadings[idx2] = " + arrayReadings[idx2] + ", arrayReadings[idx3] = " + arrayReadings[idx3]);
		logModel.debug(uniqueId + ":d12 = " + d12 + ", d23 = " + d23);
		logModel.debug(uniqueId + ":r12 = " + r12 + ", r23 = " + r23);
		
		double sinAlpha = (r12 + r23)/(d12 + d23);
		if (Math.abs(sinAlpha) > 1 && Math.abs(sinAlpha) < 1.15)
		{
			logModel.debug(uniqueId + ":acoxambrando:sinAlpha = " + sinAlpha);
			sinAlpha = FunctionsR.sign(sinAlpha)*1;
		}
		if (Math.abs(sinAlpha) > 1)
		{
			logModel.debug(uniqueId + ":sinAlpha = " + sinAlpha);
			return false;
		}
		double alpha = Math.asin(sinAlpha);
		logModel.debug(uniqueId + ":alpha = " + alpha);
	
		double d12Sigma = d12*phoErrorFront4mm; 
		double d23Sigma = d23*phoErrorFront4mm; 
		double r12Sigma = 2*sns.getReadingSigma() + d12Sigma*sinAlpha; 
		double r23Sigma = 2*sns.getReadingSigma() + d23Sigma*sinAlpha; 

		double d12Sigma2 = d12Sigma*d12Sigma; 
		double d23Sigma2 = d23Sigma*d23Sigma; 
		double r12Sigma2 = r12Sigma*r12Sigma; 
		double r23Sigma2 = r23Sigma*r23Sigma; 
		double rSigma2 = sns.getReadingSigma()*sns.getReadingSigma(); 
		
		AbstractDoubleSquareMatrix eqCovar = new DoubleSquareMatrix(4);
		AbstractDoubleVector fDeriv = new DoubleVector(4);
		
		eqCovar.setElement(0, 0, d12Sigma2); eqCovar.setElement(0, 1, d12Sigma2*sinAlpha);  eqCovar.setElement(0, 2, 0); eqCovar.setElement(0, 3, 0);
		eqCovar.setElement(1, 0, d12Sigma2*sinAlpha); eqCovar.setElement(1, 1, r12Sigma2);  eqCovar.setElement(1, 2, 0); eqCovar.setElement(1, 3, rSigma2);
		eqCovar.setElement(2, 0, 0); eqCovar.setElement(2, 1, 0);  eqCovar.setElement(2, 2, d23Sigma2); eqCovar.setElement(2, 3, d23Sigma2*sinAlpha);
		eqCovar.setElement(3, 0, 0); eqCovar.setElement(3, 1, rSigma2);  eqCovar.setElement(3, 2, d23Sigma2*sinAlpha); eqCovar.setElement(3, 3, r23Sigma2);
		
		fDeriv.setComponent(0, -r12/d12/d12);
		fDeriv.setComponent(1, 1/d12);
		fDeriv.setComponent(2, r23/d23/d23);
		fDeriv.setComponent(3, -1/d23);
		
		double fVar = eqCovar.multiply(fDeriv).scalarProduct(fDeriv);
		double fValue = r12/d12 - r23/d23;
		
		double modZ2 = Math.abs(fValue*fValue/fVar);
		
		logModel.debug(uniqueId + ":modZ2: " + modZ2 + ", fValue: " + fValue + ", fVar: " + fVar + ", wallRejectionValue2: " + wallRejectionValue2);

		if (modZ2 < wallRejectionValue2) // (<teste de hiptese>)
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	/**
	 *
	 * 1 - Prucura todas a paredes com inclinação dentro da zona de busca de inclinações.
	 * 2 - Seleciona as paredes que possuem distância a x dentro da faixa de erro.
	 * 3 - seleciona a parede com centor de massa mais próximo do sonar.
	 *
	 * @param double xR
	 * @param double yR
	 * @param double thetaR
	 * @param double reading
	 * @return
	 */
	private Wall findWall(double xR, double yR, double thetaR, double reading, double sigmaPosition2, double sigmaReading2, double sigmaTheta2)
	{
		double cosThetaR = Math.cos(thetaR), sinThetaR = Math.sin(thetaR);
		double beta = sns.getBeta(), thetaMin = 0, thetaMax = 0;
		if (sigmaPosition2 > -0.5)
		{
			double sigmaTheta = Math.sqrt(sigmaTheta2);
			thetaMin = thetaR + sns.getThetaS(idxSonar) - beta/2 - sigmaTheta;
			thetaMax = thetaMin + beta + sigmaTheta;
		}
		else
		{
			thetaMin = thetaR + sns.getThetaS(idxSonar) - beta/2 - Math.PI/2;
			thetaMax = thetaMin + beta + Math.PI/2;
		}

		double xSA = xR + sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
		double ySA = yR + sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;
		logModel.debug(uniqueId + ":findWall:(xSA,ySA):(" + xSA + "," + ySA + ")");

		logModel.debug(uniqueId + ":findWall:sigmaReading2:" + sigmaReading2 + ", sigmaPosition2:" + sigmaPosition2);
		List listWalls = null;
		if (sigmaPosition2 > -0.5)
			listWalls = map.findWalls(thetaMin, thetaMax, xSA, ySA, reading, sigmaPosition2, sigmaReading2);
		else
			listWalls = map.findWalls(thetaMin, thetaMax, xSA, ySA, reading);
		logModel.debug(uniqueId + ":findWall:listWalls.size():" + listWalls.size());
		logModel.debug(uniqueId + ":findWall:listWalls:" + listWalls);

		Wall wallMin = null;
		double d2Min = 1000000000;
		Iterator itWalls = listWalls.iterator();
		while (itWalls.hasNext())
		{
			Wall wall = (Wall) itWalls.next();
			double testValue = ((Double) itWalls.next()).doubleValue();
/*
			double dx = wall.getXMass() - xSA;
			double dy = wall.getYMass() - ySA;
			double d2 = dx*dx + dy*dy;
//*/
			if (testValue < d2Min)
			{
				wallMin = wall;
				d2Min = testValue;
			}
		}
		logModel.debug(uniqueId + ":findWall:wallMin:" + wallMin);

		return wallMin;
	}


	/**
	 *
	 * 1 - Prucura todas a paredes com inclinação dentro da zona de busca de inclinações.
	 * 2 - Seleciona as paredes que possuem distância a x dentro da faixa de erro.
	 * 3 - seleciona a parede com centor de massa mais próximo do sonar.
	 *
	 * @param double xR
	 * @param double yR
	 * @param double thetaR
	 * @param double reading
	 * @param Wall wallAssociated
	 * @return
	 */
	private boolean checkWall(double xR, double yR, double thetaR, double reading, Wall wallAssociated, double sigmaPosition2, double sigmaReading2)
	{
		double cosThetaR = Math.cos(thetaR);
		double sinThetaR = Math.sin(thetaR);

		double xSA = xR + sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
		double ySA = yR + sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;

		return map.validateWallReading(xSA, ySA, sns.getThetaS(idxSonar), thetaR, reading, wallAssociated, sigmaPosition2, sigmaReading2);
	}


	/**
	 *
	 * 1 - Prucura todas a paredes com inclinação dentro da zona de busca de inclinações.
	 * 2 - Seleciona as paredes que possuem distância a x dentro da faixa de erro.
	 * 3 - seleciona a parede com centor de massa mais próximo do sonar.
	 *
	 * @param double xR
	 * @param double yR
	 * @param double thetaR
	 * @param double reading
	 * @param Wall wall
	 * @return
	 */
	private double predictObservation(double xR, double yR, double thetaR, Wall wall)
	{
		double cosThetaR = Math.cos(thetaR);
		double sinThetaR = Math.sin(thetaR);

		double xSA = xR + sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
		double ySA = yR + sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;

		double xSW = xSA*wall.getCosTheta() + ySA*wall.getSinTheta();

		logModel.debug(uniqueId + ":predictObservation:wall: " + wall);
		logModel.debug(uniqueId + ":predictObservation:xSW: " + xSW + ", thetaR: " + thetaR);
		logModel.debug(uniqueId + ":predictObservation:thetaR: " + thetaR + ", wall.getTheta(): " + wall.getTheta());
		logModel.debug(uniqueId + ":predictObservation:FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()): " + FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()) + ", Math.PI/2: " + (Math.PI/2));

//		return Math.abs(wall.getD() - xSW);
//		return (wall.getD() - xSW);
		if (FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()) > Math.PI/2)
			return  (xSW - wall.getD());
		else
			return (wall.getD() - xSW);
	}


	private void processProduceResults(AbstractDoubleVector state, double sigmaX2, double sigmaY2, double sigmaTheta2, int idxParticle)
	{
		logModel.debug(uniqueId + ":" + idxSonar + ":produceResults:state = " + MatrixUtil.toString(state, 7, 3));
		logModel.debug(uniqueId + ":            :arrayReadings[currentIdx] = " + arrayReadings[currentIdx]);
		double sigmaPosition2 = sigmaX2 + sigmaY2, sigmaReading2 = obsCovariance.getElement(0, 0);

		if (sigmaX2 > -0.5 && arrayAssociatedWalls[idxParticle] != null && arrayMissCounter[idxParticle] <= 2)
		{
			logModel.debug(uniqueId + ":if (arrayAssociatedWalls[idxParticle] != null && arrayMissCounter[idxParticle] <= 2):arrayAssociatedWalls[idxParticle] = " + arrayAssociatedWalls[idxParticle]);
			boolean bSameWall = this.checkWall(state.getComponent(0), state.getComponent(1), state.getComponent(2), arrayReadings[currentIdx], arrayAssociatedWalls[idxParticle], sigmaPosition2, sigmaReading2);
			logModel.debug(uniqueId + ":bSameWall:" + bSameWall);
			if (bSameWall)
			{
				arrayHitCounter[idxParticle]++;
				arrayMissCounter[idxParticle] = 0;
			}
			else
			{
				arrayFailCounter[idxParticle]++;
				arrayMissCounter[idxParticle]++;
			}
		}

		if (sigmaX2 < -0.5 || arrayAssociatedWalls[idxParticle] == null || arrayMissCounter[idxParticle] > 2)
		{
			logModel.debug(uniqueId + ":if (sigmaX2 < -0.5 || arrayAssociatedWalls[idxParticle] == null || arrayMissCounter[idxParticle] > 2)");
			Wall wall = this.findWall(state.getComponent(0), state.getComponent(1), state.getComponent(2), arrayReadings[currentIdx], sigmaPosition2, sigmaReading2, sigmaTheta2);

			if (wall != null)
			{
				if (sigmaX2 < -0.5)
				{
					logModel.debug(uniqueId + ":if (wall != null) if (sigmaX2 < -0.5)");
					arrayAssociatedWalls[idxParticle] = wall;
					arrayHitCounter[idxParticle]++;
					arrayMissCounter[idxParticle] = 0;
				}
				else
				{
					logModel.debug(uniqueId + ":if (wall != null) else");
					arrayAssociatedWalls[idxParticle] = wall;
					arrayFailCounter[idxParticle] = 0;
					arrayHitCounter[idxParticle] = 1;
					arrayMissCounter[idxParticle] = 0;
				}
			}
			else
			{
				if (sigmaX2 > -0.5 || arrayMissCounter[idxParticle] > 2)
				{
					logModel.debug(uniqueId + ":else if (sigmaX2 > -0.5 || arrayMissCounter[idxParticle] > 2)");
					arrayAssociatedWalls[idxParticle] = null;
				}
				else
				{
					logModel.debug(uniqueId + ":else else");
				}
				arrayFailCounter[idxParticle]++;
				arrayHitCounter[idxParticle] = 0;
				arrayMissCounter[idxParticle]++;
			}
		}
		logModel.debug(uniqueId + ":arrayMissCounter[idxParticle]:" + arrayMissCounter[idxParticle] + ":arrayFailCounter[idxParticle]:" + arrayFailCounter[idxParticle] + ":arrayHitCounter[idxParticle]:" + arrayHitCounter[idxParticle]);
		logModel.debug(uniqueId + ":arrayAssociatedWalls[idxParticle] = " + arrayAssociatedWalls[idxParticle]);
		logModel.debug(uniqueId + ":state = " + MatrixUtil.toString(state, 7, 3));
	}
}

