package br.com.r4j.research.image.sequence.estimator;

import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.Arrays;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.research.vline.VLine;
import br.com.r4j.research.vline.VLineMap;
import br.com.r4j.robosim.MapDependent;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.robosim.model.OdometryModel;


public class VLinesDynModel implements DynamicModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, MapDependent, Configurable
{
	private static Log log = LogFactory.getLog(VLinesDynModel.class.getName());

	protected Log logEstimator = LogFactory.getLog("ukf");


	private OdometryModel odomModel = null;
	private WorldMap map = null;
	private VLineMap lineMap = null;

	private Pose2D lastMove = null;
	private AbstractDoubleVector lastPose = null;

	private AbstractDoubleSquareMatrix lastMoveCovar = null;

	private VLinesMapSensorModel mapSensorModel = null;

	private int lastStateSize = -1;


	public VLinesDynModel()
	{
		if (log.isDebugEnabled())
			log.debug("OdometryModel: nova instância");
		
		odomModel = new OdometryModel();
	}


	public String getName()
	{
		return "VLines + Odom Model";
	}

	
	public void setSensor(Sensor sns)
	{
	}


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


	public void setVLineMap(VLineMap lineMap)
	{
		this.lineMap = lineMap;
	}

	
	public void setMapSensorModel(VLinesMapSensorModel mapSensorModel)
	{
		this.mapSensorModel = mapSensorModel;
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
		// não precisa fazer nada ...
	}


	public int getDataDimension()
	{
		return odomModel.getDataDimension() + lineMap.getNumberOfStateLines()*2;
	}


	public Pose2D getLastMovement()
	{
		return lastMove;
	}


	public AbstractDoubleSquareMatrix getLastMovementCovar()
	{
		return lastMoveCovar;
	}

	
	/**
	 * O modelo de dinâmica só recebe os dados dos odômetros, copiando o modelo de odômetro e adicionando as linhas
	 * encontradas até a última iteração. Essas linhas representadas por X,Y estão armazenadas no VLineMap, que essa 
	 * classe tem referência, e pde ser acessível via algum método, que as retornam sempre na mesma ordem.
	 *
	 * A informação relevante no VLineMaps é um vetor com os ids que precisam ser removidos na iteração, e um vetor
	 * com os ids que precisam ser adicionados.
	 *
	 * O VLineMap mantém uma mapa dos ids internos das linhas para os índices iniciais das linhas no vetor de estados.
	 *
	 * Ao adicionar um linha no vetor, esses índices precisam ser atualizados.
	 *
	 * O trabalho de identificar as linhas que deve ser adicionadas / removidas fica por conta do modelo de
	 * observação, ao associar as projeções atuais às passadas e identificar quais linhas são novas, quais tem estimativa inicial
	 * para entrar no estado, quais foram perdidas, e quais são associadas ao mapa. Essa classe é mais uma casca.
	 *
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector statePredictedOutMin = odomModel.produceResults(stateBefore, sensorReadings, stateCovar);
		
		AbstractDoubleVector currPose = statePredictedOutMin;
		if (lastPose != null)
		{
			lastMove = new Pose2D(currPose);
			lastMove = lastMove.sub(lastPose);
			mapSensorModel.setCameraMovement(lastMove);
		}
		lastPose = currPose;


		AbstractDoubleVector statePredictedOut = new DoubleVector(stateBefore.dimension());
		statePredictedOut.setComponent(0, statePredictedOutMin.getComponent(0));
		statePredictedOut.setComponent(1, statePredictedOutMin.getComponent(1));
		statePredictedOut.setComponent(2, statePredictedOutMin.getComponent(2));

//		int actualLineCount = lineMap.getNumberOfStateLines();
		int actualLineCount = lastStateSize;
		if (actualLineCount > 0)
		{
//			for (int r = 3; r < this.getDataDimension(); r++)
			for (int r = 3; r < odomModel.getDataDimension() + lastStateSize*2; r++)
			{
				statePredictedOut.setComponent(r, stateBefore.getComponent(r));
			}
		}

		return statePredictedOut;
	}


	public void correctByDimension(AbstractDoubleVector stateMean, AbstractDoubleSquareMatrix stateCovar,
													AbstractDoubleVector stateMeanNew, AbstractDoubleSquareMatrix stateCovarNew)
	{
		this.updateIndexOldNewArrays();
//		logEstimator.debug("lastStateSize old: " + lastStateSize);
		this.lastStateSize = lineMap.getNumberOfStateLines();
//		logEstimator.debug("lastStateSize now: " + lastStateSize);

		int sizeOld = stateMean.dimension(), sizeNew = this.getDataDimension();

		if (log.isDebugEnabled())
		{
			log.debug("correctByDimension:stateMean: \r\n" + MatrixUtil.toString(stateMean, 9, 4));
			log.debug("correctByDimension:stateCovar: \r\n" + MatrixUtil.toString(stateCovar, 9, 4));
			log.debug("correctByDimension:stateCovarNew: \r\n" + MatrixUtil.toString(stateCovarNew, 9, 4));
			log.debug("arrayOld2New: " + Arrays.toString(arrayOld2New, 9, 4));
			log.debug("arrayNew2Line: " + Arrays.toString(arrayNew2Line));
		}

		// copia o movimento
		for (int r = 0; r < 3; r++)
				stateMeanNew.setComponent(r, stateMean.getComponent(r));
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				stateCovarNew.setElement(r, c, stateCovar.getElement(r, c));

		// copia o movimento em relação as retas já existentes
		for (int r = 0; r < 3; r++)
		{
			for (int rOld = 3; rOld < sizeOld; rOld += 2)
			{
				if (arrayOld2New[rOld] != -1)
				{
					stateCovarNew.setElement(r, arrayOld2New[rOld], stateCovar.getElement(r, rOld));
					stateCovarNew.setElement(r, arrayOld2New[rOld] + 1, stateCovar.getElement(r, rOld + 1));

					stateCovarNew.setElement(arrayOld2New[rOld], r, stateCovar.getElement(rOld, r));
					stateCovarNew.setElement(arrayOld2New[rOld] + 1, r, stateCovar.getElement(rOld + 1, r));
				}
			}
		}
		if (log.isDebugEnabled())
			log.debug("correctByDimension:(1):stateCovarNew: \r\n" + MatrixUtil.toString(stateCovarNew, 9, 4));

		// copia a relação entre retas já existentes
		for (int rOld = 3; rOld < sizeOld; rOld += 2)
		{
			if (arrayOld2New[rOld] != -1)
			{
				stateMeanNew.setComponent(arrayOld2New[rOld], stateMean.getComponent(rOld));
				stateMeanNew.setComponent(arrayOld2New[rOld] + 1, stateMean.getComponent(rOld + 1));
			}
		}
		for (int rOld = 3; rOld < sizeOld; rOld += 2)
		{
			if (arrayOld2New[rOld] != -1) for (int cOld = 3; cOld < sizeOld; cOld += 2)
			{
				if (arrayOld2New[cOld] != -1)
				{
					stateCovarNew.setElement(arrayOld2New[rOld], arrayOld2New[cOld], stateCovar.getElement(rOld, cOld));
					stateCovarNew.setElement(arrayOld2New[rOld], arrayOld2New[cOld] + 1, stateCovar.getElement(rOld, cOld + 1));
					stateCovarNew.setElement(arrayOld2New[rOld] + 1, arrayOld2New[cOld], stateCovar.getElement(rOld + 1, cOld));
					stateCovarNew.setElement(arrayOld2New[rOld] + 1, arrayOld2New[cOld] + 1, stateCovar.getElement(rOld + 1, cOld + 1));
				}
			}
		}


		if (log.isDebugEnabled())
		{
			log.debug("correctByDimension:(2):stateMeanNew: \r\n" + MatrixUtil.toString(stateMeanNew, 9, 4));
			log.debug("correctByDimension:(2):stateCovarNew: \r\n" + MatrixUtil.toString(stateCovarNew, 9, 4));
		}

		// copia a as retas novas
		for (int rNew = 3; rNew < sizeNew; rNew += 2)
		{
			if (arrayNew2Old[rNew] == -1)
			{
				VLine line = arrayNew2Line[rNew];

				stateMeanNew.setComponent(rNew, line.getRho()/line.getFocus());
				stateMeanNew.setComponent(rNew + 1, line.getBeta());

				AbstractDoubleSquareMatrix lineCovar = line.getCovar();
				if (log.isDebugEnabled())
					log.debug("lineCovar: \r\n" + MatrixUtil.toString(lineCovar, 9, 4));

				stateCovarNew.setElement(rNew, rNew, lineCovar.getElement(0, 0));
				stateCovarNew.setElement(rNew, rNew + 1, lineCovar.getElement(0, 1));
				stateCovarNew.setElement(rNew + 1, rNew, lineCovar.getElement(1, 0));
				stateCovarNew.setElement(rNew + 1, rNew + 1, lineCovar.getElement(1, 1));
			}
		}
		
		if (log.isDebugEnabled())
		{
			log.debug("correctByDimension:(3):stateMeanNew: \r\n" + MatrixUtil.toString(stateMeanNew, 9, 4));
			log.debug("correctByDimension:(3):stateCovarNew: \r\n" + MatrixUtil.toString(stateCovarNew, 9, 4));
		}
	}


	public int [] arrayOld2New = new int[100];
	public int [] arrayNew2Old = new int[100];
	public VLine [] arrayNew2Line = new VLine[100];

	private AbstractDoubleVector stateBeforeBefore = null;
	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		AbstractDoubleSquareMatrix Qmin = odomModel.getModelIncrementalCovariance(stateBefore, sensorReadings, sensorCovariance);
		double d2 = 0;
		if (stateBeforeBefore != null)
		{
			double dX = (stateBefore.getComponent(0) - stateBeforeBefore.getComponent(0));
			double dY = (stateBefore.getComponent(1) - stateBeforeBefore.getComponent(1));
			d2 = dY*dY + dX*dX;
		}
		stateBeforeBefore = stateBefore;

		if (log.isDebugEnabled())
		{
			log.debug("Qmin: \r\n" + MatrixUtil.toString(Qmin, 9, 4));
		}
		lastMoveCovar = Qmin;
		mapSensorModel.setCameraMovementCovar(sensorCovariance);

		if (lastStateSize == 0 || lastStateSize == -1)
		{
			return Qmin;
		}

		AbstractDoubleSquareMatrix QOut = new DoubleSquareMatrix(odomModel.getDataDimension() + lastStateSize*2);
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				QOut.setElement(r, c, Qmin.getElement(r, c));

		double d = Math.sqrt(d2);
		for (int r = 3; r < QOut.rows(); r += 2)
		{
			QOut.setElement(r, r, 0.4*d);
			QOut.setElement(r + 1, r + 1, 0.000008*d);
		}

		return QOut;
	}


	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		AbstractDoubleMatrix Fmin = odomModel.getTransitionMatrixJacobian(state);

//		int actualLineCount = lineMap.getNumberOfStateLines();
		if (lastStateSize == 0 || lastStateSize == -1 || state.dimension() == Fmin.rows())
		{
			return Fmin;
		}

//		AbstractDoubleSquareMatrix FOut = new DoubleSquareMatrix(odomModel.getDataDimension() + lastStateSize*2);
		AbstractDoubleSquareMatrix FOut = new DoubleSquareMatrix(state.dimension());
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				FOut.setElement(r, c, Fmin.getElement(r, c));

		for (int r = 3; r < FOut.rows(); r++)
				FOut.setElement(r, r, 1);

		return FOut;
	}


	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		if (log.isDebugEnabled())
			log.debug("sensorReadings:" + MatrixUtil.toString(sensorReadings, 6, 4));

		AbstractDoubleMatrix Sigma_min = odomModel.produceResults(sigmaIn, state, sensorReadings, sigmaError, stateCovar);

		AbstractDoubleVector currPose = new DoubleVector(3);
		currPose.setComponent(0, Sigma_min.getElement(0, 0));
		currPose.setComponent(1, Sigma_min.getElement(1, 0));
		currPose.setComponent(2, Sigma_min.getElement(2, 0));

		if (lastPose != null)
		{
			lastMove = new Pose2D(currPose);
			lastMove = lastMove.sub(lastPose);
			mapSensorModel.setCameraMovement(lastMove);
		}
		lastPose = currPose;

		int actualLineCount = lineMap.getNumberOfStateLines();
//		if (actualLineCount > 0)
		if (lastStateSize > 0)
		{
//			for (int r = 3; r < this.getDataDimension(); r++)
			for (int r = 3; r < odomModel.getDataDimension() + lastStateSize*2; r++)
			{
				for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
				{
					Sigma_min.setElement(r, idxInput, sigmaIn.getElement(r, idxInput));
				}
			}
		}

		return Sigma_min;
	}


	private int oldSize = 0;
	private void updateIndexOldNewArrays()
	{
		int actualLineCount = lineMap.getNumberOfStateLines();
//		log.debug("updateIndexOldNewArrays:actualLineCount: " + actualLineCount);
//		logEstimator.debug("updateIndexOldNewArrays:actualLineCount: " + actualLineCount);

		if (arrayOld2New.length <= actualLineCount*2 + 2)
		{
			arrayOld2New = new int [actualLineCount*2 + 2 + 100];
			arrayNew2Old = new int [actualLineCount*2 + 2 + 100];
			arrayNew2Line = new VLine[actualLineCount*2 + 2 + 100];
		}
		if (oldSize > actualLineCount)
		{
			Arrays.fill(arrayNew2Line, 3, oldSize*2, null);
			Arrays.fill(arrayOld2New, 3, oldSize*2, -1);
			Arrays.fill(arrayNew2Old, 3, oldSize*2, -1);
		}
		else
		{
			Arrays.fill(arrayNew2Line, 3, actualLineCount*2, null);
			Arrays.fill(arrayOld2New, 3, actualLineCount*2, -1);
			Arrays.fill(arrayNew2Old, 3, actualLineCount*2, -1);
		}

		Iterator itLines = lineMap.getStateLineIndexIterator();
		int countLineIdx = 3;
		while (itLines.hasNext())
		{
			VLine line = lineMap.nextLineModel(itLines);

			line.mapPrevStateVectIdx = line.mapCurrStateVectIdx;
			line.mapCurrStateVectIdx = countLineIdx;

			if (line.mapPrevStateVectIdx != -1)
				arrayOld2New[line.mapPrevStateVectIdx] = line.mapCurrStateVectIdx;
			arrayNew2Old[line.mapCurrStateVectIdx] = line.mapPrevStateVectIdx;
			arrayNew2Line[line.mapCurrStateVectIdx] = line;

			countLineIdx += 2;
		}
//		log.debug("updateIndexOldNewArrays:countLineIdx: " + countLineIdx);
//		logEstimator.debug("updateIndexOldNewArrays:countLineIdx: " + countLineIdx);
		oldSize = actualLineCount;
	}


	public boolean canProduceObservations(AbstractDoubleVector readings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar)
	{
		return true;
	}


	public void configure(PropertiesHolder propHolder, String strPath)
	{
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
	}
}

