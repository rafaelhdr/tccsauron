package br.com.r4j.robosim.model;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
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
import JSci.maths.statistics.ChiSqrDistribution;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.commons.util.Arrays;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.math.ArrayMath;
import br.com.r4j.math.FunctionsZ;
import br.com.r4j.math.geom.GeomOperations;
import br.com.r4j.math.geom.GeomUtils;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RealPoseDependent;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.BaseSonarSensor;
import br.com.r4j.robosim.MapDependent;
import br.com.r4j.robosim.Wall;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction;
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
public class SonarModel implements SensorModel, Configurable, MapDependent, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, PosteriorProbabilityDensityFunction, EstimatorRenderer, RealPoseDependent, RendererListener
// PredictedEstimateConsumer, 
{
	private static Log log = LogFactory.getLog(SonarModel.class.getName());
	private static Log logModel = LogFactory.getLog("sonarmodel");

	private WorldMap map = null;
	
	private double phoErrorFront4mm = 0.05;
	private double thetaError4mm = 0.001;
	private int idxSonar = 0;
	
	private BaseSonarSensor sns = null;


	private int k_min_readings = 0;
	private int k_max_readings = 0;
	private int k_min_dist = 0;
	private int k_max_dist = 0;
	private int k_min_dist2 = 0;
	private int k_max_dist2 = 0;

	private int currentIdx = 0;
	private int previousIdx = 0;
	private int firstIdx = 0;
	private boolean bResetObsSet = true;

	private double [] arrayReadings = null; 
	private double [][][] arrayPoses = null; 
	private double [][] arraySigma2Theta = null; 

	// indica se a observação está dentro do alcance do sonar.
	private boolean [][] arrayValid = null; 

	private boolean bAlphaValid = false;
	private int [] arrayObsLine = new int[6];


	private double perc_min_accept = 0.7;

	private double wallRejectionValue = 1.645;
	private double wallRejectionValue2 = wallRejectionValue*wallRejectionValue;

	private double rotRejectionValue = 1.0;
	private double rotRejectionValue2 = rotRejectionValue*rotRejectionValue;
	
	private Wall [] assocWall = null;
	
	private AbstractDoubleSquareMatrix obsCovariance = null; 

	private int uniqueId = 0;
	private static int uniqueIdCount = 0;

	private static int DIMENSION = 1;
	
	
	public SonarModel()
	{
		uniqueId = uniqueIdCount++;
		if (log.isDebugEnabled())
			log.debug("SonarModel: " + uniqueId);

		k_min_readings = 3;
		k_max_readings = 10;

		k_min_dist = 100;
		k_max_dist = 1000;
		k_min_dist2 = k_min_dist*k_min_dist;
		k_max_dist2 = k_max_dist*k_max_dist;

		currentIdx = 0;
		firstIdx = 0;

		arrayReadings = new double[k_max_readings];
		arraySigma2Theta = new double[1][k_max_readings];
		arrayPoses = new double[1][k_max_readings][5];
		arrayValid = new boolean[1][k_max_readings];

		assocWall = new Wall[1];

		previousIdx = arrayReadings.length;
	}
	

	public void configure(PropertiesHolder props, String strBaseKey)
	{
		if (props.containsProperty(strBaseKey + "/idx_sonar"))
			idxSonar = props.getIntegerProperty(strBaseKey + "/idx_sonar").intValue();
		if (props.containsProperty(strBaseKey + "/phoErrorFront4mm"))
			phoErrorFront4mm = props.getDoubleProperty(strBaseKey + "/phoErrorFront4mm").doubleValue();
		if (props.containsProperty(strBaseKey + "/thetaError4mm"))
			thetaError4mm = props.getDoubleProperty(strBaseKey + "/thetaError4mm").doubleValue();

		if (log.isDebugEnabled())
			log.debug("idxSonar: " + idxSonar + ", phoErrorFront4mm: " + phoErrorFront4mm);
	}


	public void setSensor(Sensor sns)
	{
		this.sns = (BaseSonarSensor) sns;
	}


	public int getSonarIndex()
	{
		return idxSonar;
	}


	public String getName()
	{
//		return "Sonar " + idxSonar + " Model ";
		return "Sonar " + idxSonar + "";
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


	public int getDataDimension()
	{
		return DIMENSION;
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
	private AbstractDoubleMatrix H = new DoubleMatrix(DIMENSION, 3);
//	private AbstractDoubleMatrix Hcovar = new DoubleMatrix(2, 3);
//	private AbstractDoubleMatrix error4covar = new DoubleMatrix(3, 3);

	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		if (!this.prepareForNewObservation(vectReadings, sensorCovariance, stateEstimate, stateCovarEstimate))
			return false;
		if (!this.validateDate())
			return false;
		if (!this.processProduceResults(stateEstimate, 
						stateCovarEstimate.getElement(0, 0), stateCovarEstimate.getElement(1, 1), stateCovarEstimate.getElement(2, 2), 
						0))
			return false;

		this.calculateTransitionMatrixJacobian(stateEstimate);
		return true;
	}


	public AbstractDoubleVector produceResults(AbstractDoubleVector state, AbstractDoubleVector snsReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector obsPredicted = new DoubleVector(getDataDimension());
		double [] arrayValues = new double[DIMENSION];
		this.predictObservation(state.getComponent(0), state.getComponent(1), state.getComponent(2), assocWall[0], arrayValues);
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":arrayValues:(" + arrayValues[0] + ")");
		obsPredicted.setComponent(0, arrayValues[0]);
//		obsPredicted.setComponent(1, arrayValues[1]);
//		obsPredicted.setComponent(2, arrayValues[2]);

		return obsPredicted;
	}


	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance)
	{
		AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(DIMENSION);
		covar.setElement(0, 0, obsCovariance.getElement(0, 0));
/*
		error4covar.setElement(2, 2, 2*obsCovariance.getElement(0, 0));
		AbstractDoubleMatrix covarRes = Hcovar.multiply(error4covar).multiply((AbstractDoubleMatrix) Hcovar.transpose());
		covar.setElement(1, 1, covarRes.getElement(0, 0));
		covar.setElement(1, 2, covarRes.getElement(0, 1));
		covar.setElement(2, 1, covarRes.getElement(1, 0));
		covar.setElement(2, 2, covarRes.getElement(1, 1));
//*/
		return covar;
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
		return H;
	}


	private void calculateTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		Wall wall = assocWall[0];
		if (wall != null)
		{
			if (FunctionsR.angularDist(state.getComponent(2) + sns.getThetaS(idxSonar), wall.getTheta()) > Math.PI/2)
			{
				H.setElement(0, 0, +wall.getCosTheta());
				H.setElement(0, 1, +wall.getSinTheta());

				double thetaMix = (state.getComponent(2) - wall.getTheta());
				double cosMix = Math.cos(thetaMix), sinMix = Math.sin(thetaMix);
				H.setElement(0, 2, -1*(sns.getXS(idxSonar)*sinMix + sns.getYS(idxSonar)*cosMix));
//				H.setElement(0, 2, 0);
			}
			else
			{
				H.setElement(0, 0, -wall.getCosTheta());
				H.setElement(0, 1, -wall.getSinTheta());

				double thetaMix = state.getComponent(2) - wall.getTheta();
				double cosMix = Math.cos(thetaMix), sinMix = Math.sin(thetaMix);
				H.setElement(0, 2, sns.getXS(idxSonar)*sinMix + sns.getYS(idxSonar)*cosMix);
//				H.setElement(0, 2, 0);
			}
		}
		else
		{
			H.setElement(0, 0, 1);
			H.setElement(0, 1, 1);
			H.setElement(0, 2, 0);
		}
/*
		double dX23 = arrayPoses[currentIdx][0] - arrayPoses[previousIdx][0];
		double dY23 = arrayPoses[currentIdx][1] - arrayPoses[previousIdx][1];
		double d23 = Math.sqrt(dX23*dX23 + dY23*dY23);
		double r23 = arrayReadings[currentIdx] - arrayReadings[previousIdx];
		double salpha = r23/d23;
		double calpha = Math.sqrt(1 - salpha*salpha);
		double d23_3 = d23*d23*d23;

		if (d23 != 0)
		{
			H.setElement(1, 0, dX23*r23/(d23_3));
			H.setElement(1, 1, dY23*r23/(d23_3));
			H.setElement(2, 0, dX23*r23/(d23_3*d23*calpha));
			H.setElement(2, 1, dY23*r23/(d23_3*d23*calpha));

			Hcovar.setElement(0, 0, H.getElement(1, 0));
			Hcovar.setElement(0, 1, H.getElement(1, 1));
			Hcovar.setElement(1, 0, H.getElement(2, 0));
			Hcovar.setElement(1, 1, H.getElement(2, 1));

		}
		else
		{
			H.setElement(1, 0, 0); H.setElement(1, 1, 0);
			H.setElement(2, 0, 0); H.setElement(2, 1, 0);

			Hcovar.setElement(0, 0, 0); Hcovar.setElement(0, 1, 0);
			Hcovar.setElement(1, 0, 0); Hcovar.setElement(1, 1, 0);
		}
		H.setElement(1, 2, 0);
		H.setElement(2, 2, 0);

		Hcovar.setElement(0, 2, 1/d23);
		Hcovar.setElement(1, 2, -r23/(d23*calpha));
//*/	
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(DIMENSION, sigmaIn.columns()); 
		double [] arrayValues = new double[DIMENSION];
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			this.predictObservation(sigmaIn.getElement(0, idxInput), 
										sigmaIn.getElement(1, idxInput), 
										sigmaIn.getElement(2, idxInput), 
										 assocWall[0], arrayValues);
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":predicted[" + idxInput + "]:" + arrayValues[0] + ", sigmaError.getElement(0, idxInput): " + sigmaError.getElement(0, idxInput));
			sigmaOut.setElement(0, idxInput, arrayValues[0] + sigmaError.getElement(0, idxInput));
/*
			logModel.debug(uniqueId + ":ukf_arrayValues:(" + arrayValues[0] + ":" + arrayValues[1] + ":" + arrayValues[2] + ")");
			obsPredictedSigma.setElement(0, idxInput, arrayValues[0] + sigmaError.getElement(0, idxInput));
			obsPredictedSigma.setElement(0, idxInput, arrayValues[1] + sigmaError.getElement(1, idxInput));
			obsPredictedSigma.setElement(0, idxInput, arrayValues[2] + sigmaError.getElement(2, idxInput));
//*/
		}
		return sigmaOut;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface PosteriorProbabilityDensityFunction

	private AbstractDoubleSquareMatrix covarInv = null;
	private double readingPF = 0;
	private double pdfConst = 0;


	public boolean canProduceObservationsPF(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		this.prepareForNewObservation(vectReadings, sensorCovariance, stateEstimate, stateCovarEstimate);
		boolean bCanProduceObs = this.validateDate();
		if (bCanProduceObs)
		{
/*
			this.calculateTransitionMatrixJacobian(stateEstimate);
			for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
				error4covar.setElement(i, j, 2*stateCovar.getElement(i, j));
//*/
		}
		return bCanProduceObs;
	}


	public void setAdditiveNoise(AbstractDoubleSquareMatrix obsCovarianceTmp)
	{
		this.obsCovariance = new DoubleSquareMatrix(DIMENSION);
		obsCovariance.setElement(0, 0, obsCovarianceTmp.getElement(0, 0));
//		error4covar.setElement(2, 2, 2*obsCovarianceTmp.getElement(0, 0));
/*
		AbstractDoubleMatrix covarRes = Hcovar.multiply(error4covar).multiply((AbstractDoubleMatrix) Hcovar.transpose());
		obsCovariance.setElement(1, 1, covarRes.getElement(0, 0));
		obsCovariance.setElement(1, 2, covarRes.getElement(0, 1));
		obsCovariance.setElement(2, 1, covarRes.getElement(1, 0));
		obsCovariance.setElement(2, 2, covarRes.getElement(1, 1));
//*/
		covarInv = obsCovariance.inverse();
		pdfConst = 1.0/Math.sqrt(((AbstractDoubleSquareMatrix) obsCovariance.scalarMultiply(2*Math.PI)).det());
		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":obsCovariance: \r\n" + MatrixUtil.toString(obsCovariance, 9, 4));
			logModel.debug(uniqueId + ":covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
			logModel.debug(uniqueId + ":pdfConst: " + pdfConst);
		}
		if (pdfConst < 1)
			pdfConst = 1;
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":pdfConst(2): " + pdfConst);
	}


	public void copyParticleData(int idxFrom, int idxTo)
	{
		if (assocWall.length <= idxFrom)
			this.changeArrayWallSize(idxFrom + 1);

		if (assocWall.length <= idxTo)
			this.changeArrayWallSize(idxTo + 1);

		assocWall[idxTo] = assocWall[idxFrom];
	}


	public void setObservations(AbstractDoubleVector readings)
	{
		this.readingPF = readings.getComponent(0);
	}

	
	public double stateProbability(AbstractDoubleMatrix inputStates, int rowObjectCount)
	{
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":stateProbability:" +  rowObjectCount);
		if (assocWall.length >= rowObjectCount)
			this.changeArrayWallSize(rowObjectCount + 1);

		AbstractDoubleVector state = MatrixUtil.getRow(inputStates, rowObjectCount);


		if (assocWall[rowObjectCount] == null || this.findWall(rowObjectCount, 5*5, 5*5, true) == null)
		{
			assocWall[rowObjectCount] = this.findWall(rowObjectCount, 5*5, 5*5, false);
		}
		
		if (assocWall[rowObjectCount] != null)
		{
			double [] arrayValues = new double[3];
			this.predictObservation(state.getComponent(0), state.getComponent(1), state.getComponent(2), 
											assocWall[rowObjectCount], arrayValues);
			double diff = arrayValues[0] - readingPF;
			double expoent = diff*covarInv.getElement(0, 0)*diff;
			logModel.warn(uniqueId + ":diff: " + diff + ", expoent: " + expoent + ", pdfConst: " + pdfConst);
			logModel.warn(uniqueId + ":pdfConst*Math.exp(-1.0/2.0*expoent): " + pdfConst*Math.exp(-1.0/2.0*expoent));
			return pdfConst*Math.exp(-1.0/2.0*expoent);
		}
		else
		{
			logModel.warn(uniqueId + ":stateProbability: retornando 0 (zero !!)!!");
			return 0;
		}
	}	



//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Método internos
	
	private void changeArrayWallSize(int sizeNew)
	{
		Wall [] assocWallTmp = new Wall[sizeNew];
		for (int i = 0; i < sizeNew && i < assocWall.length; i++)
		{
			assocWallTmp[i] = assocWall[i];
		}

		assocWall = assocWallTmp;
	}


	/**
	 *  1- Faz update quando uma nova observação vai ser tratada.
	 */
	protected boolean prepareForNewObservation(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector statePredicted, AbstractDoubleSquareMatrix stateCovar)
	{
		bAlphaValid = false;
		if (vectReadings.getComponent(idxSonar) == -1)
			return false;

		previousIdx = currentIdx;
		currentIdx = (currentIdx + 1)%arrayReadings.length;

		if (logModel.isDebugEnabled())
			logModel.debug("idxSonar: " + idxSonar + ", currentIdx: " + currentIdx + ", previousIdx: " + previousIdx + ", firstIdx: " + firstIdx);

		arrayReadings[currentIdx] = vectReadings.getComponent(idxSonar);

		arrayPoses[0][currentIdx][0] = statePredicted.getComponent(0); 
		arrayPoses[0][currentIdx][1] = statePredicted.getComponent(1); 
		arrayPoses[0][currentIdx][2] = statePredicted.getComponent(2);
		arrayPoses[0][currentIdx][3] = Math.cos(arrayPoses[0][currentIdx][2]);
		arrayPoses[0][currentIdx][4] = Math.sin(arrayPoses[0][currentIdx][2]);
		arraySigma2Theta[0][currentIdx] = stateCovar.getElement(2, 2);
		if (sns.isOutOfRange(arrayReadings[currentIdx]))
			arrayValid[0][currentIdx] = false;
		else
			arrayValid[0][currentIdx] = true;

		if (obsCovariance == null)
			obsCovariance = new DoubleSquareMatrix(1);
		obsCovariance.setElement(0, 0, sensorCovariance.getElement(idxSonar, idxSonar));

		if (bResetObsSet)
		{
			bResetObsSet = false;
			firstIdx = currentIdx;
			return false;
		}
		else
		{
			// Determina se deve ou não coletar mais pontos, e se já tem o suficiente.

			int diff = FunctionsZ.diffCircular(currentIdx, firstIdx, arrayReadings.length) + 1;
			if (logModel.isDebugEnabled())
				logModel.debug("diff: " + diff + ", k_min_readings: " + k_min_readings + ", k_max_readings: " + k_max_readings);
			if (diff < k_min_readings)
				return false;
			if (diff >= k_max_readings)
				firstIdx = (firstIdx + 1)%arrayReadings.length;

			double dx = arrayPoses[0][currentIdx][0] - arrayPoses[0][firstIdx][0];
			double dy = arrayPoses[0][currentIdx][1] - arrayPoses[0][firstIdx][1];
			double d2 = dx*dx + dy*dy;

			if (logModel.isDebugEnabled())
			{
				logModel.debug("firstIdx: " + firstIdx + ", arrayPoses[0][firstIdx][0]: " + arrayPoses[0][firstIdx][0]+ ", arrayPoses[0][firstIdx][1]: " + arrayPoses[0][firstIdx][1]);
				logModel.debug("dx: " + dx + ", dy: " + dy);
				logModel.debug("d2: " + d2 + ", k_min_dist2: " + k_min_dist2 + ", k_max_dist2: " + k_max_dist2);
			}

			if (d2 < k_min_dist2)
				return false;
			if (d2 >= k_max_dist2)
				firstIdx = (firstIdx + 1)%arrayReadings.length;

		}

		return true;
	}


	/**
	 * 2 - Verifica se com a última medição é possível gerar algum resultado.
	 */
	protected boolean validateDate()
	{
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":(validateDate)testing arrayReadings[currentIdx]: " + arrayReadings[currentIdx] + ", arrayReadings[currentIdx] != -1: " + (arrayReadings[currentIdx] != -1) + ", idxSonar = " + idxSonar);

		// Testa para ver se o robo não rodou.
		// Precisa levar em conta a distância percorrida (ou não?).

		double dThetaPrevious = arrayPoses[0][firstIdx][2], dThetaCurrent = arrayPoses[0][currentIdx][2];
		double dX = arrayPoses[0][(currentIdx - 1 + arrayReadings.length)%arrayReadings.length][0] - arrayPoses[0][currentIdx][0];
		double dY = arrayPoses[0][(currentIdx - 1 + arrayReadings.length)%arrayReadings.length][1] - arrayPoses[0][currentIdx][1];
		double d2Dist = dX*dX + dY*dY;
		double dSigma2ThetaPrevious = arraySigma2Theta[0][firstIdx], dSigma2ThetaCurrent = arraySigma2Theta[0][currentIdx];
		double sigma2Rot = dSigma2ThetaPrevious > dSigma2ThetaCurrent ? dSigma2ThetaPrevious : dSigma2ThetaCurrent;
		double rotDist = FunctionsR.angularDist(dThetaPrevious, dThetaCurrent);
		if (logModel.isDebugEnabled())
		{
			logModel.debug("d2Dist: " + d2Dist + ", d2Dist*thetaError4mm*thetaError4mm: " + d2Dist*thetaError4mm*thetaError4mm);
			logModel.debug("dThetaPrevious: " + dThetaPrevious + ", dThetaCurrent: " + dThetaCurrent);
			logModel.debug("rotDist: " + rotDist + ", sigma2Rot: " + sigma2Rot + ", rotRejectionValue2: " + rotRejectionValue2 + ", dSigma2ThetaCurrent: " + dSigma2ThetaCurrent);
		}
//		if (rotDist*rotDist > sigma2Rot*rotRejectionValue2)
		if (rotDist*rotDist > d2Dist*thetaError4mm*thetaError4mm*rotRejectionValue2)
		{
			if (logModel.isDebugEnabled())
				logModel.debug("não valido");
			bResetObsSet = true;
			for (int i = 0; i < assocWall.length; i++)
				assocWall[i] = null;

			return false;
		}

		// Verifica se a leitura é de alguma coisa
		if (sns.isOutOfRange(arrayReadings[currentIdx]))
			return false;
		
		// Verifica se tem um número mínimo de leituras válidas
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":");
		int countValid = 0;
		if (logModel.isDebugEnabled())
		{
			for (int i = firstIdx; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
			{
				countValid += arrayValid[0][i] ? 1 : 0;
				logModel.debug("\t" + arrayValid[0][i]);
			}
		}
		else
			for (int i = firstIdx; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
				countValid += arrayValid[0][i] ? 1 : 0;

		double diff = FunctionsZ.diffCircular(currentIdx, firstIdx, arrayReadings.length) + 1;
		double perc = 1.0*countValid / (1.0*diff);
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":diff: " + diff + ", countValid: " + countValid + ", perc: " + perc + ", currentIdx: " + currentIdx + ", firstIdx: " + firstIdx + ", perc_min_accept: " + perc_min_accept);

		// Se tem um número mínimo de leituras, verifica se elas podem corresponder a 
		// uma parede.
		if (perc > perc_min_accept && diff >= k_min_readings)
			return this.testForWall();
		else
		{
			assocWall[0] = null;
			return false;
		}
	}


	/**
	 * 2.1 - Verifica se as leituras correspondem a uma parede.
	 */
	private boolean testForWall()
	{
		int diff = FunctionsZ.diffCircular(currentIdx, firstIdx, arrayReadings.length);
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":(testForWall)diff: " + diff);
		for (int i = firstIdx; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
			diff -= arrayValid[0][i] ? 0 : 1;
		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":(testForWall)diff_2: " + diff);

		if (diff + 1 < k_min_readings)
		{
			assocWall[0] = null;
			return false;
		}

		double [] arrayGamma = new double[diff];
		boolean bBefore = false;
		int iBefore = 0;
		double thetaMean = 0; int countUsed = 0;
		String str_gamma = "", str_r = "";

		for (int i = firstIdx, j = 0; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
		{
			if (logModel.isDebugEnabled())
			{
				logModel.debug("i: " + i + ", pose: " + arrayPoses[0][i][0] + ", " + arrayPoses[0][i][1] + ", range: " + arrayReadings[i]);
			}
		}

		for (int i = firstIdx, j = 0; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
		{
			if (arrayValid[0][i])
				{thetaMean += arrayPoses[0][i][2]; countUsed++;}

			if (bBefore && arrayValid[0][i])
			{
//				int iminus = (i-1 + arrayReadings.length)%arrayReadings.length;
				double dX12 = (double) (arrayPoses[0][i][0] - arrayPoses[0][iBefore][0]);
				double dY12 = (double) (arrayPoses[0][i][1] - arrayPoses[0][iBefore][1]);
				double d12 = Math.sqrt(dX12*dX12 + dY12*dY12);
				double r12 = (double) (arrayReadings[i] - arrayReadings[iBefore]);

				if (logModel.isDebugEnabled())
				{
					logModel.debug("i: " + i + ", iBefore: " + iBefore);
					logModel.debug("d12: " + d12 + ", r12: " + r12);
				}

				arrayGamma[j] = r12 / d12;
				if (logModel.isDebugEnabled())
					str_gamma += arrayGamma[j] + ", ";

				j++;
			}
			if (arrayValid[0][i])
			{
				bBefore = true;
				iBefore = i;
				if (logModel.isDebugEnabled())
					str_r += arrayReadings[i] + ", ";
			}
			else
			{
//				bBefore = false;
			}
		}
		thetaMean /= countUsed;
		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":thetaMean: " + thetaMean);
			logModel.debug("readings: " + str_r);
			logModel.debug("gammas: " + str_gamma);
		}
		
		double gammaMean = ArrayMath.mean(arrayGamma);
		ArrayMath.sub(arrayGamma, gammaMean);
		double var4zeromean = ArrayMath.variance(arrayGamma, 0);
		if (logModel.isDebugEnabled())
			logModel.debug("gammaMean:" + gammaMean + ", var4zeromean:" + var4zeromean);
		
		double dX = (double) (arrayPoses[0][currentIdx][0] - arrayPoses[0][firstIdx][0]);
		double dY = (double) (arrayPoses[0][currentIdx][1] - arrayPoses[0][firstIdx][1]);
		double d = Math.sqrt(dX*dX + dY*dY);
		double dr = (double) (arrayReadings[currentIdx] - arrayReadings[firstIdx]);

		double sinAlpha = dr/d;
		if (logModel.isDebugEnabled()) logModel.debug(uniqueId + ":sinAlpha:" + sinAlpha);
		if (Math.abs(sinAlpha) > 1)
		{
			assocWall[0] = null;
			return false;
		}

		double alpha = Math.asin(sinAlpha);

		if (logModel.isDebugEnabled())
			logModel.debug(uniqueId + ":alpha = " + alpha);

		double s2d = d*phoErrorFront4mm/(diff - 1); s2d = s2d*s2d;
		double s2r = sns.getReadingSigma()*sns.getReadingSigma(); 
		if (logModel.isDebugEnabled())
			logModel.debug("s2d: " + s2d + ", s2r: " + s2r + ", phoErrorFront4mm: " + phoErrorFront4mm);

		AbstractDoubleSquareMatrix eqCovar = new DoubleSquareMatrix(2);
		eqCovar.setElement(0, 0, s2d); eqCovar.setElement(0, 1, s2d*sinAlpha);
		eqCovar.setElement(1, 0, s2d*sinAlpha); eqCovar.setElement(1, 1, 2*s2r + s2d*sinAlpha);
		
		AbstractDoubleVector fDeriv = new DoubleVector(2);
		fDeriv.setComponent(0, -diff*1.0*dr/(d*d));
		fDeriv.setComponent(1, diff*1.0/d);

		double fVar = eqCovar.multiply(fDeriv).scalarProduct(fDeriv);
		double chiStat = (diff - 1)*var4zeromean/fVar;
		ChiSqrDistribution chiDist = new ChiSqrDistribution(diff - 1);
		double chi09 = chiDist.inverse(0.1);
		double chi05 = chiDist.inverse(0.55);
		double chi03 = chiDist.inverse(0.7);
		if (logModel.isDebugEnabled())
		{
			logModel.debug("chi03: " + chi03 + ", chi05: " + chi05 + ", chi09: " + chi09 + ", diff: " + diff);
			logModel.debug("eqCovar: \r\n" + MatrixUtil.toString(eqCovar, 9, 4));
			logModel.debug("fDeriv: \r\n" + MatrixUtil.toString(fDeriv, 9, 4));
			logModel.debug(uniqueId + ":chiStat: " + chiStat + ", fVar: " + fVar + ", wallRejectionValue2: " + wallRejectionValue2);
		}
		if (chiStat < chi03) //chi05) //wallRejectionValue2) // (<teste de hiptese>)
		{
			bAlphaValid = true;

			double cosThetaR = Math.cos(thetaMean), sinThetaR = Math.sin(thetaMean);
			double cosThetaS = Math.cos(thetaMean + sns.getThetaS(idxSonar)), sinThetaS = Math.sin(thetaMean + sns.getThetaS(idxSonar));
//			double alpha = Math.asin(sinAlpha);
			double cosAlpha = Math.sqrt(1 - sinAlpha*sinAlpha);

			double cosWallTmp = cosThetaR*cosAlpha + sinThetaR*sinAlpha;
			double sinWallTmp = sinThetaR*cosAlpha - cosThetaR*sinAlpha;

			double cosWall = cosWallTmp*0 - sinThetaR*1;
			double sinWall = sinWallTmp*0 + cosThetaR*1;
			double thetaWall = Math.asin(sinWall);

			double cosWallNormal = cosWall*0 - sinWall*1;
			double sinWallNormal = sinWall*0 + cosWall*1;

			double xSA_base = sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
			double ySA_base = sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;
			double xSA_last = arrayPoses[0][currentIdx][0] + xSA_base, ySA_last = arrayPoses[0][currentIdx][1] + ySA_base;
			double dSA_last = Math.sqrt(xSA_last*xSA_last + ySA_last*ySA_last);

			double sinThetaP = ySA_last / dSA_last;
			double cosThetaP = xSA_last / dSA_last;
			double cosWall_min_P = cosWall*cosThetaP + sinWall*sinThetaP;

			int signReading = FunctionsR.angularDist(thetaMean + sns.getThetaS(idxSonar), thetaWall) < Math.PI/2 ? 1 : -1;

//			double r_wall = arrayReadings[currentIdx] + xSA_last*cosAlpha + ySA_last*sinAlpha;
//			double r_wall = arrayReadings[currentIdx] + xSA_last*cosWall + ySA_last*sinWall;
			double r_wall = signReading*arrayReadings[currentIdx] + dSA_last*cosWall_min_P;

			logModel.debug("xSA_base: " + xSA_base + ", ySA_base: " + ySA_base);
			logModel.debug("xSA_last: " + xSA_last + ", ySA_last: " + ySA_last + ", dSA_last: " + dSA_last);
			logModel.debug("cosThetaR: " + cosThetaR + ", sinThetaR: " + sinThetaR);
			logModel.debug("cosThetaS: " + cosThetaS + ", sinThetaS: " + sinThetaS + ", tethaS: " + (thetaMean + sns.getThetaS(idxSonar)));
			logModel.debug("r_wall: " + r_wall + ", sinWall: " + sinWall + ", cosWall: " + cosWall + ", thetaWall: " + thetaWall);
			logModel.debug("signReading*arrayReadings[currentIdx]: " + signReading*arrayReadings[currentIdx] + ", dSA_last*cosWall_min_P: " + dSA_last*cosWall_min_P + ", cosWall_min_P: " + cosWall_min_P);
			logModel.debug("alpha: " + alpha + ", sinAlpha: " + sinAlpha + ", cosAlpha: " + cosAlpha);

			arrayObsLine[0] = (int) (r_wall*cosWall);
			arrayObsLine[1] = (int) (r_wall*sinWall);

			if (Math.abs(sinWall) < 0.001 || sinWall < cosWall)
			{
				arrayObsLine[2] = (int) (r_wall*cosWall - 55000);
				arrayObsLine[3] = (int) (r_wall*sinWall - 55000*sinWallNormal/cosWallNormal);
				arrayObsLine[4] = (int) (r_wall*cosWall + 55000);
				arrayObsLine[5] = (int) (r_wall*sinWall + 55000*sinWallNormal/cosWallNormal);
			}
			else
			{
				arrayObsLine[2] = (int) (r_wall*cosWall - 55000*cosWallNormal/sinWallNormal);
				arrayObsLine[3] = (int) (r_wall*sinWall - 55000);
				arrayObsLine[4] = (int) (r_wall*cosWall + 55000*cosWallNormal/sinWallNormal);
				arrayObsLine[5] = (int) (r_wall*sinWall + 55000);
			}

			return true;
		}
		else
		{
			int cutCount = (int) ((diff + 0.5)*perc_min_accept);
//			int diffSmall = diff - cutCount;
			if (cutCount < k_min_readings - 1)
				cutCount = k_min_readings - 1;
			double [] arrayGammaCut = new double[cutCount];

			if (logModel.isDebugEnabled())
				logModel.debug("cutCount: " + cutCount + ", diff: " + diff);
			if (logModel.isDebugEnabled())
				logModel.debug("arrayGamma: " + Arrays.toString(arrayGamma));
			Arrays.sort(arrayGamma);
			if (logModel.isDebugEnabled())
				logModel.debug("arrayGamma sorted: " + Arrays.toString(arrayGamma));
			int l = 0, r = arrayGamma.length - 1;
			int toRemove = diff - cutCount;
			while (toRemove-- > 0)
			{
				if (Math.abs(arrayGamma[l]) > Math.abs(arrayGamma[r]))
					l++;
				else
					r--;
			}
			for (int i = 0; i < arrayGammaCut.length; i++)
				arrayGammaCut[i] = arrayGamma[i + l];
			if (logModel.isDebugEnabled())
				logModel.debug("arrayGammaCut: " + Arrays.toString(arrayGammaCut));

			double gammaMeanZeroCut = ArrayMath.mean(arrayGammaCut);
			double var4zeromeanCut = ArrayMath.variance(arrayGammaCut, gammaMeanZeroCut);
			if (logModel.isDebugEnabled())
				logModel.debug("gammaMeanZeroCut = " + gammaMeanZeroCut + ", var4zeromeanCut:" + var4zeromeanCut);

			double chiStatCut = (cutCount - 1)*var4zeromeanCut/fVar;
			ChiSqrDistribution chiDistCut = new ChiSqrDistribution(cutCount - 1);
			double chi09Cut = chiDistCut.inverse(0.1);
			double chi05Cut = chiDistCut.inverse(0.5);
			double chi03Cut = chiDistCut.inverse(0.7);
			if (logModel.isDebugEnabled())
			{
				logModel.debug("chi05Cut: " + chi05Cut + ", chi09Cut: " + chi09Cut);
				logModel.debug(uniqueId + ":chiStatCut: " + chiStatCut + ", wallRejectionValue2: " + wallRejectionValue2);
			}

			// Se aprovado no segundo teste, não valida o teste, mas não limpa a referência associada.
			if (chiStatCut < chi03Cut) //wallRejectionValue2) // (<teste de hiptese>)
			{
				logModel.debug("aprovado em chiStatCut < wallRejectionValue2");
			}
			else
			{
				logModel.debug("rejeitado em chiStatCut < wallRejectionValue2");
				assocWall[0] = null;
				int diffNew = FunctionsZ.diffCircular(currentIdx, firstIdx, arrayReadings.length);
				if (diffNew > k_min_readings - 1)
					diffNew = k_min_readings - 1;
				firstIdx = (currentIdx - diffNew + arrayReadings.length)%arrayReadings.length;
			}
			return false;
		}
	}


	/**
	 * 3. Associação. Associa uma paredeàs observações.
	 * Retorna falso caso a associaçãonão ocorra ou as últimas observações 
	 * não observam a parede.
	 */
	private boolean processProduceResults(AbstractDoubleVector state, double sigmaX2, double sigmaY2, double sigmaTheta2, int idxParticle)
	{
		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":" + idxSonar + ":produceResults:state = " + MatrixUtil.toString(state, 7, 3));
			logModel.debug(uniqueId + ":            :arrayReadings[currentIdx] = " + arrayReadings[currentIdx]);
		}
		double sigmaPosition2 = sigmaX2 + sigmaY2, sigmaReading2 = obsCovariance.getElement(0, 0);

		// Verifica se uma parede já assciada continua sendo observada.
		if (assocWall[idxParticle] != null)
		{
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":if (assocWall[idxParticle] != null):assocWall[idxParticle] = " + assocWall[idxParticle]);
			Wall wallOk = this.findWall(idxParticle, sigmaPosition2, sigmaReading2, true);
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":wallOk:" + wallOk);
			if (wallOk == null)
				return false;
		}

		// Procura associar uma parede caso nenhuma esteja associada ou se a antiga associada
		// já caducou.
		if (assocWall[idxParticle] == null)
		{
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":if (assocWall[idxParticle] == null)");
			Wall wall = this.findWall(idxParticle, sigmaPosition2, sigmaReading2, false);

			if (wall != null)
			{
				if (logModel.isDebugEnabled())
					logModel.debug(uniqueId + ":if (wall != null)");
				assocWall[idxParticle] = wall;
				return true;
			}
			else
				return false;
		}
		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":assocWall[idxParticle] = " + assocWall[idxParticle]);
			logModel.debug(uniqueId + ":state = " + MatrixUtil.toString(state, 7, 3));
		}
		return true;
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
	private Wall findWall(int idxParticle, double sigmaReading2, double sigmaPosition2, boolean bJustCheck)
	{
		double thetaMean = 0; int countUsed = 0;
		for (int i = firstIdx, j = 0; i != currentIdx; i = (i + 1)%arrayReadings.length, j++)
		{
			if (arrayValid[0][i])
			{
				thetaMean += arrayPoses[0][i][2];
				countUsed++;
			}
		}
		thetaMean /= countUsed;
		double cosThetaR = Math.cos(thetaMean), sinThetaR = Math.sin(thetaMean);
		logModel.debug("findWall:thetaMean: " + thetaMean + ", cosThetaR: " + cosThetaR);

		double xSA_base = sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
		double ySA_base = sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;
		double xSA_first = arrayPoses[0][firstIdx][0] + xSA_base, ySA_first = arrayPoses[0][firstIdx][1] + ySA_base;
		double xSA_bef_last = arrayPoses[0][(currentIdx - 1 + arrayReadings.length)%arrayReadings.length][0] + xSA_base, 
			   ySA_bef_last = arrayPoses[0][(currentIdx - 1 + arrayReadings.length)%arrayReadings.length][1] + ySA_base;
		double xSA_last = arrayPoses[0][currentIdx][0] + xSA_base, ySA_last = arrayPoses[0][currentIdx][1] + ySA_base;

		if (logModel.isDebugEnabled())
		{
			logModel.debug("xSA_base|ySA_base:" + xSA_base + "|" + ySA_base);
			logModel.debug("xSA_first|ySA_first:" + xSA_first + "|" + ySA_first);
			logModel.debug("xSA_last|ySA_last:" + xSA_last + "|" + ySA_last);
		}

		double beta = sns.getBeta();
		double sigmaTheta = Math.sqrt(arraySigma2Theta[0][currentIdx]);
		double thetaMin = thetaMean + sns.getThetaS(idxSonar) - beta/2 - sigmaTheta;
		double thetaMax = thetaMean + sns.getThetaS(idxSonar) + beta/2 + sigmaTheta;

		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":findWall:idxSonar:(getXS(idxSonar),getYS(idxSonar),getThetaS(idxSonar)):" + idxSonar + ":(" + sns.getXS(idxSonar) + "," + sns.getYS(idxSonar) + ", " + sns.getThetaS(idxSonar) + ")");
			logModel.debug(uniqueId + ":findWall:(xSA_last, ySA_last):(" + xSA_last + "," + ySA_last + ")");
			logModel.debug(uniqueId + ":findWall:sigmaReading2:" + sigmaReading2 + ", sigmaPosition2:" + sigmaPosition2);
		}

		double thetaS = thetaMean + sns.getThetaS(idxSonar);
		if (!bJustCheck)
		{
			// Primeiro filtro: pela inclinação da parede e pela posição do robô na últmia posição
			// (se está na frente ou atras da parede).
			List listWalls = map.findWalls(thetaMin, thetaMax, xSA_last, ySA_last);
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":findWall:listWalls:" + listWalls);
			if (listWalls.size() == 0)
				return null;

			double sinThetaS = Math.sin(thetaS), cosThetaS = Math.cos(thetaS);
			if (logModel.isDebugEnabled())
				logModel.debug("thetaS:" + thetaS + ", cosThetaS:" + cosThetaS + ", sinThetaS:" + sinThetaS);
			double xSide_first = xSA_first - 10000*cosThetaS, ySide_first = ySA_first - 10000*sinThetaS;
			double xOtherSide_first = xSA_first + 10000*cosThetaS, yOtherSide_first = ySA_first + 10000*sinThetaS;
			double xSide_last = xSA_last - 10000*cosThetaS, ySide_last = ySA_last - 10000*sinThetaS;
			double xOtherSide_last = xSA_last + 10000*cosThetaS, yOtherSide_last = ySA_last + 10000*sinThetaS;


			if (logModel.isDebugEnabled())
			{
				logModel.debug("xSide_first|ySide_first:" + xSide_first + "|" + ySide_first);
				logModel.debug("xOtherSide_first|yOtherSide_first:" + xOtherSide_first + "|" + yOtherSide_first);
				logModel.debug("xSide_last|ySide_last:" + xSide_last + "|" + ySide_last);
				logModel.debug("xOtherSide_last|yOtherSide_last:" + xOtherSide_last + "|" + yOtherSide_last);
			}

			ArrayList listWallsAproved = new ArrayList();

			Iterator itWalls = listWalls.iterator();
			while (itWalls.hasNext())
			{
				Wall wall = (Wall) itWalls.next();
				if (logModel.isDebugEnabled())
					logModel.debug("testing wall:    " + wall);

				// Segundo filtro: Se a primeira e última observações intersectam a parede.
				if (GeomOperations.intersect(xSide_first, ySide_first, xOtherSide_first, yOtherSide_first, 
											 wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2()) &&
					GeomOperations.intersect(xSide_last, ySide_last, xOtherSide_last, yOtherSide_last, 
											 wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2()))
				{			

					// Terceiro filtro: Se a primeira, penultima e última observações correspondem a parede.
					double dWall_first = xSA_first*wall.getCosTheta() + ySA_first*wall.getSinTheta();
					double dWall_bef_last = xSA_bef_last*wall.getCosTheta() + ySA_bef_last*wall.getSinTheta();
					double dWall_last = xSA_last*wall.getCosTheta() + ySA_last*wall.getSinTheta();
					double value_first = 0, value_bef_last = 0, value_last = 0;

					if (FunctionsR.angularDist(thetaS, wall.getTheta()) > Math.PI/2)
					{
						value_first = Math.abs((dWall_first - wall.getD()) - arrayReadings[firstIdx]);
						value_bef_last = Math.abs((dWall_bef_last - wall.getD()) - arrayReadings[(currentIdx - 1 + arrayReadings.length)%arrayReadings.length]);
						value_last = Math.abs((dWall_last - wall.getD()) - arrayReadings[currentIdx]);
					}
					else
					{
						value_first = Math.abs((wall.getD() - dWall_first) - arrayReadings[firstIdx]);
						value_bef_last = Math.abs((wall.getD() - dWall_bef_last) - arrayReadings[(currentIdx - 1 + arrayReadings.length)%arrayReadings.length]);
						value_last = Math.abs((wall.getD() - dWall_last) - arrayReadings[currentIdx]);
					}
					if (logModel.isDebugEnabled())
					{
						logModel.debug("value_first:    " + value_first);
						logModel.debug("value_bef_last: " + value_bef_last);
						logModel.debug("value_last:     " + value_last);
					}

					double sigmaError2 = sigmaPosition2 + sigmaReading2;
					double zValue2_first = value_first*value_first/sigmaError2;
					double zValue2_bef_last = value_bef_last*value_bef_last/sigmaError2;
					double zValue2_last = value_last*value_last/sigmaError2;
					if (logModel.isDebugEnabled())
						logModel.debug("zValue2_bef_last: " + zValue2_bef_last + ", sigmaError2: " + sigmaError2 + ", wallRejectionValue2: " + wallRejectionValue2);
					if (zValue2_first < wallRejectionValue2*100 && 
						zValue2_bef_last < wallRejectionValue2*100 && 
						zValue2_last < wallRejectionValue2*100)
					{
						listWallsAproved.add(wall);
						listWallsAproved.add(new Double(zValue2_bef_last + zValue2_last + zValue2_first));
					}
				}
				else
				{
					if (logModel.isDebugEnabled())
						logModel.debug("rejected by intersect");
				}

			}
			if (listWallsAproved.size() == 0)
				return null;

			// Quarto filtro: Pega o menor (difrente da tese).
			Wall wallMin = null;
			double d2Min = 1000000000;
			Iterator itWallApprovds = listWallsAproved.iterator();
			while (itWallApprovds.hasNext())
			{
				Wall wall = (Wall) itWallApprovds.next();
				double testValue = ((Double) itWallApprovds.next()).doubleValue();
				if (testValue < d2Min)
				{
					wallMin = wall;
					d2Min = testValue;
				}
			}
			if (logModel.isDebugEnabled())
				logModel.debug(uniqueId + ":findWall:wallMin:" + wallMin);

			return wallMin;
		}
		else
		{
			// Terceiro filtro: Se a primeira, penultima e última observações correspondem a parede.
			double dWall_first = xSA_first*assocWall[idxParticle].getCosTheta() + ySA_first*assocWall[idxParticle].getSinTheta();
			double dWall_bef_last = xSA_bef_last*assocWall[idxParticle].getCosTheta() + ySA_bef_last*assocWall[idxParticle].getSinTheta();
			double dWall_last = xSA_last*assocWall[idxParticle].getCosTheta() + ySA_last*assocWall[idxParticle].getSinTheta();
			double value_first = 0, value_bef_last = 0, value_last = 0;

			if (FunctionsR.angularDist(thetaS, assocWall[idxParticle].getTheta()) > Math.PI/2)
			{
				value_first = Math.abs((dWall_first - assocWall[idxParticle].getD()) - arrayReadings[firstIdx]);
				value_bef_last = Math.abs((dWall_bef_last - assocWall[idxParticle].getD()) - arrayReadings[(currentIdx - 1 + arrayReadings.length)%arrayReadings.length]);
				value_last = Math.abs((dWall_last - assocWall[idxParticle].getD()) - arrayReadings[currentIdx]);
			}
			else
			{
				value_first = Math.abs((assocWall[idxParticle].getD() - dWall_first) - arrayReadings[firstIdx]);
				value_bef_last = Math.abs((assocWall[idxParticle].getD() - dWall_bef_last) - arrayReadings[(currentIdx - 1 + arrayReadings.length)%arrayReadings.length]);
				value_last = Math.abs((assocWall[idxParticle].getD() - dWall_last) - arrayReadings[currentIdx]);
			}
			if (logModel.isDebugEnabled())
			{
				logModel.debug("value_first:    " + value_first);
				logModel.debug("value_bef_last: " + value_bef_last);
				logModel.debug("value_last:     " + value_last);
			}

			double sigmaError2 = sigmaPosition2 + sigmaReading2;
			double zValue2_first = value_first*value_first/sigmaError2;
			double zValue2_bef_last = value_bef_last*value_bef_last/sigmaError2;
			double zValue2_last = value_last*value_last/sigmaError2;
			if (logModel.isDebugEnabled())
				logModel.debug("zValue2_bef_last: " + zValue2_bef_last + ", sigmaError2: " + sigmaError2 + ", wallRejectionValue2: " + wallRejectionValue2);

			if (zValue2_first < wallRejectionValue2*100 && 
				zValue2_bef_last < wallRejectionValue2*100 && 
				zValue2_last < wallRejectionValue2*100)
				return assocWall[idxParticle];
			else
				return null;
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
	 * @param Wall wall
	 * @return
	 */
	private void predictObservation(double xR, double yR, double thetaR, Wall wall, double [] arrayValues)
	{
		thetaR = arrayPoses[0][currentIdx][2];

		double cosThetaR = Math.cos(thetaR);
		double sinThetaR = Math.sin(thetaR);

		double xSA = xR + sns.getXS(idxSonar)*cosThetaR - sns.getYS(idxSonar)*sinThetaR;
		double ySA = yR + sns.getYS(idxSonar)*cosThetaR + sns.getXS(idxSonar)*sinThetaR;

		double dWall = xSA*wall.getCosTheta() + ySA*wall.getSinTheta();

		if (logModel.isDebugEnabled())
		{
			logModel.debug(uniqueId + ":predictObservation:wall: " + wall);
			logModel.debug(uniqueId + ":predictObservation:dWall: " + dWall + ", thetaR: " + thetaR);
			logModel.debug(uniqueId + ":predictObservation:thetaR: " + thetaR + ", wall.getTheta(): " + wall.getTheta());
			logModel.debug(uniqueId + ":predictObservation:FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()): " + FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()) + ", Math.PI/2: " + (Math.PI/2));
		}

		if (FunctionsR.angularDist(thetaR + sns.getThetaS(idxSonar), wall.getTheta()) > Math.PI/2)
			arrayValues[0] = (dWall - wall.getD());
		else
			arrayValues[0] = (wall.getD() - dWall);
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
		double dX = meanPred.getComponent(0) - meanCorr.getComponent(0);
		double dY = meanPred.getComponent(1) - meanCorr.getComponent(1);
		double dTheta = meanPred.getComponent(2) - meanCorr.getComponent(2);
/*
		double dX = arrayPoses[0][currentIdx][0] - mean.getComponent(0);
		double dY = arrayPoses[0][currentIdx][1] - mean.getComponent(1);
		double dTheta = arrayPoses[0][currentIdx][2] - mean.getComponent(2);
//*/
		for (int i = firstIdx, j = 0; i != (currentIdx + 1)%arrayReadings.length; i = (i + 1)%arrayReadings.length)
		{
			arrayPoses[0][i][0] -= dX;
			arrayPoses[0][i][1] -= dY;
			arrayPoses[0][i][2] -= dTheta;
		}
	}


	private Pose2D poseRobot = null;
	private int currentStep = -1;
	private ArrayList listPoses = new ArrayList();
	private ArrayList listReadings = new ArrayList();
	private boolean bRendering = false;

	private Wall wallLast = null;

	
	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
	}


	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
	}


	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(20);} catch (Exception e) {}
		}
		listPoses.add(pose);
		currentStep = listPoses.size() - 1;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;

		Graphics2D g2d = e.getGraphics();
		if (bAlphaValid)
		{
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.red);
			g2d.drawLine(arrayObsLine[2], arrayObsLine[3], arrayObsLine[4], arrayObsLine[5]);
			g2d.fillOval(arrayObsLine[0] - 15, arrayObsLine[1] - 15, 30, 30);
			g2d.drawLine(0, 0, arrayObsLine[0], arrayObsLine[1]);
		}
		if (assocWall[0] != null)
		{
			wallLast = assocWall[0];
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.yellow);
			g2d.drawLine((int) wallLast.getX1()+10, (int) wallLast.getY1()+10, (int) wallLast.getX2()+10, (int) wallLast.getY2()+10);
			g2d.drawLine((int) wallLast.getX1()-10, (int) wallLast.getY1()-10, (int) wallLast.getX2()-10, (int) wallLast.getY2()-10);
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		Graphics2D g2d = e.getGraphics();
		g2d.setXORMode(Color.white);
		if (bAlphaValid)
		{
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.red);
			g2d.drawLine(arrayObsLine[2], arrayObsLine[3], arrayObsLine[4], arrayObsLine[5]);
			g2d.fillOval(arrayObsLine[0] - 15, arrayObsLine[1] - 15, 30, 30);
		}
		if (assocWall[0] != null)
		{
			wallLast = assocWall[0];
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.yellow);
			g2d.drawLine((int) wallLast.getX1(), (int) wallLast.getY1(), (int) wallLast.getX2(), (int) wallLast.getY2());
			g2d.drawLine(0, 0, arrayObsLine[0], arrayObsLine[1]);
		}
		bRendering = false;
	}


	public void erase(RendererEvent e)
	{
		bRendering = true;
		Graphics2D g2d = e.getGraphics();
		g2d.setXORMode(Color.white);
		if (bAlphaValid)
		{
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.red);
			g2d.drawLine(arrayObsLine[2], arrayObsLine[3], arrayObsLine[4], arrayObsLine[5]);
			g2d.fillOval(arrayObsLine[0] - 15, arrayObsLine[1] - 15, 30, 30);
		}
		if (wallLast != null)
		{
			wallLast = assocWall[0];
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(Color.yellow);
			g2d.drawLine((int) wallLast.getX1(), (int) wallLast.getY1(), (int) wallLast.getX2(), (int) wallLast.getY2());
			g2d.drawLine(0, 0, arrayObsLine[0], arrayObsLine[1]);
		}
		bRendering = false;
	}


	public void actionCompleted(RobotPlayerEvent e)
	{
	}


	public void endOfActions(RobotPlayerEvent e)
	{
	}


	public void beginOfActions(RobotPlayerEvent e)
	{
	}


	public void actionsUpdated(RobotPlayerEvent e)
	{
	}


	public void setRealPose(Pose2D realPose)
	{
		this.poseRobot = realPose;
	}

}

