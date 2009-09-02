package br.com.r4j.research.image.sequence.estimator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.image.sequence.featurematch.vline.*;
import br.com.r4j.research.vline.*;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.robosim.model.RadarModel;

import br.com.r4j.robosim.estimator.*;
import br.com.r4j.robosim.estimator.impl.*;
import br.com.r4j.robosim.realrobot.*;
import br.com.r4j.robosim.model.*;

import java.util.*;
import java.awt.Color;
import java.awt.image.BufferedImage;

import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.commons.util.ColorSequence;
import br.com.r4j.commons.util.ImageUtil;


public class VLinesMapSensorModel implements SensorModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, Configurable
{
	private static Log log = LogFactory.getLog(VLinesMapSensorModel.class.getName());
	private static Log logStats = LogFactory.getLog("estatisticas");


	private WorldMap map = null;
	private VLineMap lineMap = null;
	private CameraModel camModel = null;

	private VLineMapSensor sns = null;
	private VLinesDynModel dynModel = null;
	private DifferenceSimpleMatchingPhaseMatcher matchingPhase = null;

	private LineMatchingResult matchngResult = null;

	private boolean bBeingUsed = false;
	private boolean bUseMap = true;

	// Geram a estimativa do movimento apenas.
	private AditiveNosieExtendedKalmanFilter movementFilter = null;
	private OdometryModel odomModel = null;
	private FixedOdoSensor odoSensor = null;
	private int countParams = 0;


	public VLinesMapSensorModel(DifferenceSimpleMatchingPhaseMatcher matchingPhase)
	{
		this.matchingPhase = matchingPhase;
		this.bUseMap = true;
		try
		{
			movementFilter = new AditiveNosieExtendedKalmanFilter();
			odomModel = new OdometryModel();
			odoSensor = new FixedOdoSensor();
			movementFilter.setDynamicModel(odomModel, odoSensor);
			movementFilter.setState(new DoubleVector(3), new DoubleSquareMatrix(3));
			countParams = 0;
		}
		catch (Exception e)
		{
			log.error("dig dig dig", e);
		}
	}


	public void setCameraMovement(Pose2D lastMove)
	{
		if (log.isDebugEnabled())
			log.debug("setCameraMovement(Pose2D lastMove): " + lastMove);
		odoSensor.setReadings(lastMove);
		countParams++;
		if (countParams == 2)
		{
			countParams = 0;
			odoSensor.dataAvailable();
			odomModel.dataAvailable();
			movementFilter.estimate(false);
			if (log.isDebugEnabled())
				log.debug("movementFilter.estimate(false);");
		}
	}


	public void setCameraMovementCovar(AbstractDoubleSquareMatrix lastMoveCovar)
	{
		if (log.isDebugEnabled())
			log.debug("setCameraMovementCovar(AbstractDoubleSquareMatrix lastMoveCovar): " + lastMoveCovar);
		odoSensor.setCovariance(lastMoveCovar);
/*
		countParams++;
		if (countParams == 2)
		{
			countParams = 0;
			odoSensor.dataAvailable();
			odomModel.dataAvailable();
			movementFilter.estimate(false);
			log.debug("movementFilter.estimate(false);");
		}
//*/
	}


	public LineExtrationResult getLastResult()
	{
		return sns.newLineProjs;
	}

	
	public LineMatchingResult getMatchingResults()
	{
		return matchngResult;
	}


	public void setSensor(Sensor sns)
	{
		this.sns = (VLineMapSensor) sns;
	}


	public CameraModel getCameraModel()	{return camModel;}
	public void setCameraModel(CameraModel camModel)
	{
		this.camModel = camModel;
		matchingPhase.setCameraModel(camModel);
	}
	public void setWorldMap(WorldMap map)
	{
		this.map = map;
		matchingPhase.setWorldMap(map);
	}

	public VLineMap getLineMap()	{return this.lineMap;}
	public VLineMap getVLineMap()	{return this.lineMap;}
	public void setVLineMap(VLineMap lineMap)
	{
		this.lineMap = lineMap;
		matchingPhase.setLineMap(lineMap);
	}
	public void setDynModel(VLinesDynModel dynModel)
	{
		this.dynModel = dynModel;
	}


	public String getName()
	{
//		return "VLines Map Sensor Model";
		return "Visão";
	}

	
	public void configure(PropertiesHolder props, String strBaseKey)
	{
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
	}


	public boolean isBeingUsed()
	{
		return bBeingUsed;
	}
	public void setBeingUsed(boolean bBeingUsed)
	{
		this.bBeingUsed = bBeingUsed;
	}


	public void setUseMap(boolean bUseMap)
	{
		this.bUseMap = bUseMap;
	}


	private RigidBodyTranformation2D cameraPoseEstimateLastNotTempered = null;
	private RigidBodyTranformation2D cameraPoseEstimateLast = null;

	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
		if (cameraPoseEstimateLast != null)
		{
			Pose2D pose2d = cameraPoseEstimateLast.getAsPose();
			double dX = meanPred.getComponent(0) - meanCorr.getComponent(0);
			double dY = meanPred.getComponent(1) - meanCorr.getComponent(1);
			double dTheta = meanPred.getComponent(2) - meanCorr.getComponent(2);

			cameraPoseEstimateLast = new RigidBodyTranformation2D(pose2d.getX() - dX, pose2d.getY() - dY, pose2d.getTheta() - dTheta);
		}
	}


	private int [] arrayOutDebug = null;
	private int itCount = 0; // de-de-debug!
	public void doDataAssociation(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		bBeingUsed = true;

		LineExtrationResult extrationResult = sns.getLastResult();

		long start_t = System.currentTimeMillis();

		RigidBodyTranformation2D cameraTrafo = new RigidBodyTranformation2D(stateEstimate);
		AbstractDoubleSquareMatrix cameraPoseCovar = stateCovarEstimate;

		Pose2D cameraMoveEstimate = dynModel.getLastMovement();
		AbstractDoubleSquareMatrix cameraMoveCovar = dynModel.getLastMovementCovar();

		AbstractDoubleVector moveMean = movementFilter.getMean();
		AbstractDoubleSquareMatrix moveCovar = movementFilter.getCovariance();
		movementFilter.setState(new DoubleVector(3), new DoubleSquareMatrix(3));

		/*PRINTIMGS
		log.debug("doDataAssociation (map)");
		log.debug("robot pose: " + stateEstimate);
		log.debug("robot move: " + cameraMoveEstimate);
		log.debug("moveMean: \r\n" + MatrixUtil.toString(moveMean, 9, 4));
		log.debug("moveCovar: \r\n" + MatrixUtil.toString(moveCovar, 9, 4));
		//*/

		long tmTm = System.currentTimeMillis();
		matchingPhase.setItCount(itCount);
		if (cameraPoseEstimateLast != null)
		{
			cameraMoveEstimate = cameraTrafo.getAsPose().sub(cameraPoseEstimateLast.getAsPose());
			cameraMoveEstimate = cameraPoseEstimateLast.inverseRotatePosition(cameraMoveEstimate);
			Pose2D cameraMoveEstimateNotTempered = cameraTrafo.getAsPose().sub(cameraPoseEstimateLastNotTempered.getAsPose());
			cameraMoveEstimateNotTempered = cameraPoseEstimateLastNotTempered.inverseRotatePosition(cameraMoveEstimateNotTempered);
			/*PRINTIMGS
			log.debug("cameraMoveEstimate: " + cameraMoveEstimate);
			log.debug("cameraMoveEstimateNotTempered: " + cameraMoveEstimateNotTempered);
			//*/

			matchngResult = matchingPhase.lineMatching(sns.getImage(), sns.getImageWidth(), sns.getImageHeight(), 
														cameraTrafo, cameraPoseCovar, 
														cameraMoveEstimate, 
														moveCovar, sns.getLastResult(), bUseMap);
		}
		else
		{
			matchngResult = matchingPhase.lineMatching(sns.getImage(), sns.getImageWidth(), sns.getImageHeight(), 
														cameraTrafo, cameraPoseCovar, 
														new Pose2D(moveMean), 
														moveCovar, sns.getLastResult(), bUseMap);
		}

		if (log.isDebugEnabled())
			log.debug("matching: " + (System.currentTimeMillis() - tmTm));
		cameraPoseEstimateLast = cameraTrafo;
		cameraPoseEstimateLastNotTempered = cameraTrafo;

		/*PRINTIMGS
		if (log.isDebugEnabled() && matchngResult != null)
		{
			if (arrayOutDebug == null || arrayOutDebug.length < sns.imgData.length)
				arrayOutDebug = new int [sns.imgData.length];
			br.com.r4j.commons.util.Arrays.arrayCopy(sns.imgData, arrayOutDebug);

			for (int i = 0; i < sns.newLineProjs.arrayProjMeasures.length; i++)
			{
				boolean bIsNewLine = false;
				VLineProj lineProj = sns.newLineProjs.arrayProjMeasures[i];
				LineSegmentCollection lCol = lineProj.getSegment();
				VLine line = matchngResult.arrayProjMeasuresLines[i];
				Color clr = null;
				clr = lineMap.getDebugColor(line);
				if (clr == null)
				{
					bIsNewLine = true;
					clr = ColorSequence.getColor(lineMap.nextColorIdx++, 0);
					lineMap.setDebugColor(line, clr);
				}
				int clrVal = ThreeBandPackedUtil.getPackedPixel(clr);

				for (int v = lCol.yIni; v <= lCol.yEnd; v++)
				{
					int u = (int) (lCol.mModel*v + lCol.xModel);
					if (u < 0) u = 0;
					if (u >= sns.imgWidth) u = sns.imgWidth - 1;
					arrayOutDebug[v*sns.imgWidth + u] = clrVal;
				}

				int lineMark = line.mapIdx*10%(lCol.yEnd - lCol.yIni) + lCol.yIni;
				int uMid = (int) (lCol.mModel*lineMark + lCol.xModel);
				if (bIsNewLine) for (int j = uMid - 3; j < uMid + 4; j++)
				{
					if (j >= 0 && j < sns.imgWidth)
						arrayOutDebug[lineMark*sns.imgWidth + j] = 0xFFFFFF;

				}
				else for (int j = uMid - 3; j < uMid + 4; j++)
				{
					if (j >= 0 && j < sns.imgWidth)
						arrayOutDebug[lineMark*sns.imgWidth + j] = clrVal;

				}
			}
			if (itCount < 10)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,sns.imgWidth,sns.imgHeight,BufferedImage.TYPE_INT_RGB), "debA_00" + itCount + ".jpg");
			else if (itCount < 100)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,sns.imgWidth,sns.imgHeight,BufferedImage.TYPE_INT_RGB), "debA_0" + itCount + ".jpg");
			else
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,sns.imgWidth,sns.imgHeight,BufferedImage.TYPE_INT_RGB), "debA_" + itCount + ".jpg");
		}
		//*/
		itCount++;

		if (log.isDebugEnabled())
			log.debug("getDataDimension(): " + getDataDimension());
		bUseMap = true;
	}


	public int getDataDimension()
	{
		return lineMap.getNumberOfMappedLines();
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector state, AbstractDoubleVector snsReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector obsPredicted = new DoubleVector(getDataDimension());

		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		double zRotCamera = camModel.getZRotCamera();
		double xOffset = camModel.getXOffset();
		double yOffset = camModel.getYOffset();
		double [] arrayLineTrafo = new double [2];

		double xRobot = state.getComponent(0);
		double yRobot = state.getComponent(1);
		double thetaRobot = state.getComponent(2);


		RigidBodyTranformation2D robotTrafo = new RigidBodyTranformation2D(xRobot, yRobot, thetaRobot);
		log.debug("xOffset: (before):" + xOffset);
		robotTrafo.directRotateLine(xOffset, yOffset, arrayLineTrafo);
		log.debug("xOffset:  (after):" + xOffset);
		RigidBodyTranformation2D cameraTrafo = new RigidBodyTranformation2D(xRobot + arrayLineTrafo[0], yRobot + arrayLineTrafo[1], thetaRobot + zRotCamera);
		if (log.isDebugEnabled())
			log.debug("cameraTrafo: " + cameraTrafo);

		int lIdx = 0;
		Iterator itLines = lineMap.getMappedLineIndexIterator();
		for (; lIdx < lineMap.getNumberOfMappedLines(); lIdx++)
		{
			VLine line = lineMap.nextLine(itLines);
			VLineRef lineRef = lineMap.getAssocVLineRef(line);
			double xLine = lineRef.getX();
			double yLine = lineRef.getY();
//			log.debug("map(before trafo): xLine: " + xLine + ", yLine: " + yLine);
			cameraTrafo.inverseLine(xLine, yLine, arrayLineTrafo);
//			log.debug("map (after trafo): xLine: " + arrayLineTrafo[0] + ", yLine: " + arrayLineTrafo[1]);

			double uEst = uCenter;
			if (Math.abs(arrayLineTrafo[0]) > 0.01)
				uEst = uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0];

			/*PRINTIMGS
			log.debug("uCenter - f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (uCenter - f * arrayLineTrafo[1] / arrayLineTrafo[0]));
			log.debug("uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]));
			log.debug("-uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (-uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]));
			//*/

			obsPredicted.setComponent(lIdx, uEst);
		}
		return obsPredicted;
	}


	/**
	 *  A verificação é feita a nivel de sensor.
	 */
	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		return true;
	}


	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance)
	{
		AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(lineMap.getNumberOfMappedLines());

		int countLineIdx = 0;
		Iterator itLines = lineMap.getMappedLineIndexIterator();
		while (itLines.hasNext())
		{
			VLine line = lineMap.nextLine(itLines);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);

			covar.setElement(countLineIdx, countLineIdx, lineProj.getUErrorSigma());
			countLineIdx++;
		}

		return covar;
	}


	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings)
	{
		return sensorReadings;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface EKFDoubleVectorFunction

	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
//		AbstractDoubleMatrix H = new DoubleMatrix(state.dimension(), lineMap.getNumberOfMappedLines());
		AbstractDoubleMatrix H = new DoubleMatrix(lineMap.getNumberOfMappedLines(), state.dimension());

		/*PRINTIMGS
		log.debug("getDataDimension():" + getDataDimension());
		log.debug("lineMap.getNumberOfMappedLines():" + lineMap.getNumberOfMappedLines());
		log.debug("state.dimension():" + state.dimension());
		//*/

		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		double zRotCamera = camModel.getZRotCamera();

		double dX = state.getComponent(0), dY = state.getComponent(1);
		double dTh = state.getComponent(2) + zRotCamera;
		double sinTh = Math.sin(dTh), cosTh = Math.cos(dTh);

		/* Diferenciação de:
			VLineRef lineRef = lineMap.getAssocVLineRef(line);
			double xLine = lineRef.getX(), yLine = lineRef.getY();
			cameraTrafo.inverseLine(xLine, yLine, arrayLineTrafo);
			uEst = uCenter - f * arrayLineTrafo[1] / arrayLineTrafo[0];
					\/  -> sehila perque ...
			uEst = uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0];
		*/

//		log.debug("H calc: state: " + MatrixUtil.toString(state, 7, 4));
		int lIdx = 0;
		Iterator itLines = lineMap.getMappedLineIndexIterator();
		for (; lIdx < lineMap.getNumberOfMappedLines(); lIdx++)
		{
			VLine line = lineMap.nextLine(itLines);
			VLineRef lineRef = lineMap.getAssocVLineRef(line);
			double xLine = lineRef.getX();
			double yLine = lineRef.getY();
//			log.debug("H calc: lineRef: " + lineRef);

//			double V = cosTh*(xLine - dX) + sinTh*(yLine - dY);
//			double Z = cosTh*(yLine - dY) - sinTh*(xLine - dX);
			double V = cosTh*(yLine - dY) - sinTh*(xLine - dX);
			double Z = cosTh*(xLine - dX) + sinTh*(yLine - dY);
//			log.debug("H calc: V: " + V + ", Z: " + Z);
			double Z2 = Z*Z;

			if (Z2 > 0.01)
			{
				// Menos na frente de todos devido a subtração do uCenter ...
/*
				H.setElement(lIdx, 0, -f*(sinTh*Z + cosTh*V)/Z2);
				H.setElement(lIdx, 1, f*(cosTh*Z - sinTh*V)/Z2); 
				H.setElement(lIdx, 2, f*(V*V/Z2 + 1));
/*/
// with corr ...
				H.setElement(lIdx, 0, f*(sinTh*Z + cosTh*V)/Z2);
				H.setElement(lIdx, 1, -f*(cosTh*Z - sinTh*V)/Z2); 
				H.setElement(lIdx, 2, -f*(V*V/Z2 + 1));
//*/
//				H.setElement(lIdx, 3 + lIdx*2, -H.getElement(lIdx, 0));
//				H.setElement(lIdx, 3 + lIdx*2 + 1, -H.getElement(lIdx, 1));

			}
			else
			{
				if (log.isDebugEnabled())
					log.debug("Z == 0");
			}
			if (log.isDebugEnabled())
				log.debug("H: \r\n" + MatrixUtil.toString(H, 9, 6));
		}
		return H;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		double zRotCamera = camModel.getZRotCamera();
		double xOffset = camModel.getXOffset();
		double yOffset = camModel.getYOffset();
		double [] arrayLineTrafo = new double [2];
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(getDataDimension(), sigmaIn.columns()); 
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			double xRobot = sigmaIn.getElement(0, idxInput);
			double yRobot = sigmaIn.getElement(1, idxInput);
			double thetaRobot = sigmaIn.getElement(2, idxInput);
			RigidBodyTranformation2D robotTrafo = new RigidBodyTranformation2D(xRobot, yRobot, thetaRobot);
			robotTrafo.directRotateLine(xOffset, yOffset, arrayLineTrafo);
			log.debug("xOffset: (before):" + xOffset);
			RigidBodyTranformation2D cameraTrafo = new RigidBodyTranformation2D(xRobot + arrayLineTrafo[0], yRobot + arrayLineTrafo[1], thetaRobot + zRotCamera);
//			log.debug("cameraTrafo: " + cameraTrafo);

			int lIdx = 0;
			Iterator itLines = lineMap.getMappedLineIndexIterator();
			for (; lIdx < lineMap.getNumberOfMappedLines(); lIdx++)
			{
				VLine line = lineMap.nextLine(itLines);
				VLineRef lineRef = lineMap.getAssocVLineRef(line);
				double xLine = lineRef.getX();
				double yLine = lineRef.getY();
//				log.debug("map(before trafo): xLine: " + xLine + ", yLine: " + yLine);
				cameraTrafo.inverseLine(xLine, yLine, arrayLineTrafo);
//				log.debug("map (after trafo): xLine: " + arrayLineTrafo[0] + ", yLine: " + arrayLineTrafo[1]);

				double uEst = uCenter;
				if (Math.abs(arrayLineTrafo[0]) > 0.01)
					uEst = uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0];

				/*PRINTIMGS
				log.debug("uCenter - f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (uCenter - f * arrayLineTrafo[1] / arrayLineTrafo[0]));
				log.debug("uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]));
				log.debug("-uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]: " + (-uCenter + f * arrayLineTrafo[1] / arrayLineTrafo[0]));
				//*/

				sigmaOut.setElement(lIdx, idxInput, uEst + sigmaError.getElement(lIdx, idxInput));
			}
		}
		return sigmaOut;
	}

}

