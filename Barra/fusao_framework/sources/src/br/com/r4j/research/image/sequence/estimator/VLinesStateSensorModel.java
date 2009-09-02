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

import java.util.*;
import java.awt.Color;
import java.awt.image.BufferedImage;

import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.commons.util.ColorSequence;
import br.com.r4j.commons.util.ImageUtil;


public class VLinesStateSensorModel implements SensorModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, Configurable
{
	private static Log log = LogFactory.getLog(VLinesStateSensorModel.class.getName());


	private WorldMap map = null;
	private VLineMap lineMap = null;
	private CameraModel camModel = null;

	private VLineStateSensor sns = null;
	private VLinesDynModel dynModel = null;
	private DifferenceSimpleMatchingPhaseMatcher matchingPhase = null;

	private VLinesMapSensorModel lineMapModel = null;


	public VLinesStateSensorModel(DifferenceSimpleMatchingPhaseMatcher matchingPhase)
	{
		this.matchingPhase = matchingPhase;
	}


	public void setVLinesMapSensorModel(VLinesMapSensorModel lineMapModel)
	{
		this.lineMapModel = lineMapModel;
	}
	
	
	public void setSensor(Sensor sns)
	{
		this.sns = (VLineStateSensor) sns;
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
		return "VLines State Sensor Model";
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


	private int [] arrayOutDebug = null;
	private int itCount = 0; // de-de-debug!
	public void doDataAssociation(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		log.debug("doDataAssociation (state)");

		if (!lineMapModel.isBeingUsed())
		{
			lineMapModel.setUseMap(false);
			lineMapModel.doDataAssociation(stateEstimate, stateCovarEstimate);
			lineMapModel.setBeingUsed(false);
		}

		LineExtrationResult extrationResult = lineMapModel.getLastResult();

		long start_t = System.currentTimeMillis();

		RigidBodyTranformation2D cameraTrafo = new RigidBodyTranformation2D(stateEstimate);
		AbstractDoubleSquareMatrix cameraPoseCovar = stateCovarEstimate;
		log.debug("robot pose: " + stateEstimate);

		Pose2D cameraMoveEstimate = dynModel.getLastMovement();
		AbstractDoubleSquareMatrix cameraMoveCovar = dynModel.getLastMovementCovar();
		log.debug("robot move: " + cameraMoveEstimate);

		long tmTm = System.currentTimeMillis();
		matchingPhase.setItCount(itCount);
		matchingPhase.updateFutureLineStates(lineMapModel.getMatchingResults(), cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar);
		log.debug("matching: " + (System.currentTimeMillis() - tmTm));

		itCount++;
	}


	public int getDataDimension()
	{
		return lineMap.getNumberOfStateLines();
//		return sns.getDataDimension();
	}


	private AbstractDoubleVector stateLast = null;

	public void correctAngulation(AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector stateCorr = new DoubleVector(state.dimension());
		if (stateLast == null)
			log.debug("stateLast == null");
		log.debug("state: \r\n" + MatrixUtil.toString(state, 7, 4));

		double f = camModel.getUFocusDistance(), uCenter = camModel.getUAxisPixelCenter();

		double duMeasSum = 0, duEstMax = 0, dTotal = 0, dDiff = 0, du_RotThTot = 0;
		Iterator itLines = lineMap.getStateLineIndexIterator();
		for (int lIdx = 0; lIdx < lineMap.getNumberOfStateLines(); lIdx++)
		{
			VLine line = lineMap.nextLineModel(itLines);
			VLineProj lineProj1 = lineMap.getBeforeLastMeasuredProjection(line);
			VLineProj lineProj2 = lineMap.getLastMeasuredProjection(line);
			Pose2D poseP1 = lineMap.getBeforeLastMeasuredRigidBodyTranformation2D(line).getAsPose();
			double dX = state.getComponent(0) - poseP1.getX();
			double dY = state.getComponent(1) - poseP1.getY();
			double dTh = state.getComponent(2) - poseP1.getTheta();
			double du_RotTh = dTh*camModel.getUAxisPixelCount()/camModel.getVisibilityRangeAngle();
			log.debug("dX = " + dX + ", dY = " + dY + ", dTh = " + dTh + ", du_RotTh = " + du_RotTh);
			log.debug("lineProj1.getU() = " + lineProj1.getU() + ", lineProj2.getU() = " + lineProj2.getU());

			// uCenter ta certo assim mesmo. Não segue a orientação de sempre.
			double duEstMaxWNoRot = (-f*dY + (lineProj1.getU() - uCenter)*dX)/3000;
			double duMeas = lineProj2.getU() - lineProj1.getU();
			double dVal = Math.abs(lineProj1.getU() - uCenter);
			double ddTotal = 1 - Math.abs(dVal*dVal)/(camModel.getUAxisPixelCount()*camModel.getUAxisPixelCount()/4);
			double ddDiff = duMeas - du_RotTh;
			log.debug("du_RotTh = " + du_RotTh + ", duMeas = " + duMeas + ". ddTotal = " + ddTotal + ", ddDiff: " + ddDiff);

			if (dVal < 101)
			{
				duEstMax += duEstMaxWNoRot*ddTotal;

				du_RotThTot += du_RotTh*ddTotal;
				duMeasSum += duMeas*ddTotal;

				dTotal += ddTotal;
				dDiff += ddDiff*ddTotal;
			}
		}
		double uCorr = (duMeasSum - duEstMax)/dTotal;
//		double uCorr = (duMeasSum - duEstMax)/lineMap.getNumberOfStateLines();
		log.debug("duMeasSum = " + duMeasSum + ", duEstMax = " + duEstMax + ". uCorr = " + uCorr);
		log.debug("duMeasSum/dTotal = " + duMeasSum/dTotal + ", duEstMax/dTotal = " + duEstMax/dTotal + ", dTotal = " + dTotal);

		double dRad = uCorr * camModel.getVisibilityRangeAngle() / camModel.getUAxisPixelCount();
		log.debug("dRad = " + dRad + ", getVisibilityRangeAngle = " + camModel.getVisibilityRangeAngle() + ". getUAxisPixelCount = " + camModel.getUAxisPixelCount());

		double du_ThTot = du_RotThTot*camModel.getVisibilityRangeAngle()/camModel.getUAxisPixelCount()/dTotal;
		log.debug("du_ThTot/dTotal = " + du_ThTot);
		double weightInv = 1.8;
		if (Math.abs(du_RotThTot) > Math.abs(duMeasSum))
		{
			double du_rot_corr = (duMeasSum - du_RotThTot)/dTotal;
			double th_rot_corr = du_rot_corr*camModel.getVisibilityRangeAngle()/camModel.getUAxisPixelCount();
			log.debug("du_rot_corr: " + du_rot_corr + ", th_rot_corr: " + th_rot_corr);
			double thDiff = dDiff*camModel.getVisibilityRangeAngle()/camModel.getUAxisPixelCount();
			log.debug("dDiff: " + dDiff + ", thDiff: " + thDiff);
			log.debug("dDiff/dTotal: " + dDiff/dTotal + ", thDiff/dTotal: " + thDiff/dTotal);
			log.debug("((dTotal + 1.8)*state.getComponent(2) + thDiff*dTotal)/(dTotal + weightInv): " + ((dTotal + weightInv)*state.getComponent(2) + (thDiff/dTotal)*dTotal)/(dTotal + weightInv));

			state.setComponent(0, state.getComponent(0));
			state.setComponent(1, state.getComponent(1));
//			state.setComponent(2, ((dTotal + weightInv)*state.getComponent(2) + (thDiff/dTotal)*dTotal)/(dTotal + weightInv));
			state.setComponent(2, state.getComponent(2));
		}
		else
		{
			state.setComponent(0, state.getComponent(0));
			state.setComponent(1, state.getComponent(1));
			state.setComponent(2, state.getComponent(2));
		}
//*/
/*
		for (int i = 3; i < state.dimension(); i++)
			stateCorr.setComponent(i, state.getComponent(i));
		return stateCorr;
//*/
	}


	public void setLastState(AbstractDoubleVector state)
	{
		this.stateLast = state;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector state, AbstractDoubleVector snsReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		log.debug("VLineStateSensor->produceResults");
		AbstractDoubleVector obsPredicted = new DoubleVector(getDataDimension());

		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		double xRobotActual = state.getComponent(0), yRobotActual = state.getComponent(1);
		double thetaRobotActual = state.getComponent(2);
		log.debug("xRobotActual: " + xRobotActual + ", yRobotActual: " + yRobotActual + ", thetaRobotActual: " + thetaRobotActual);

		Iterator itLines = lineMap.getStateLineIndexIterator();
		for (int lIdx = 0; lIdx < lineMap.getNumberOfStateLines(); lIdx++)
		{
			VLine line = lineMap.nextLineModel(itLines);
			RigidBodyTranformation2D rbTrafoIni = lineMap.getLineModelRigidBodyTranformation2D(line);
			log.debug("line: " + line + ", rbTrafoIni: " + rbTrafoIni);

			double [] arrayLineTrafo = new double[2];
			rbTrafoIni.inverseLine(xRobotActual, yRobotActual, arrayLineTrafo);

			double xRobot = arrayLineTrafo[0], yRobot = arrayLineTrafo[1];
			double thetaRobot = thetaRobotActual - rbTrafoIni.getAsPose().getTheta();
			double sin_th = Math.sin(thetaRobot), cos_th = Math.cos(thetaRobot);

			double rho_0_lin = state.getComponent(3 + lIdx*2);
			double beta_0 = state.getComponent(3 + lIdx*2 + 1);
			double rho_0 = f * rho_0_lin;
			double sin_beta_0 = Math.sin(beta_0), cos_beta_0 = Math.cos(beta_0);

			double dxx = rho_0*cos_beta_0 - xRobot, dyy = rho_0*sin_beta_0 - yRobot;
			double T2 = xRobot*xRobot + yRobot*yRobot;
			double biyy = (dyy*cos_th - dxx*sin_th), bixx = (dxx*cos_th + dyy*sin_th);
			log.debug("rho_0: " + rho_0 + ", beta_0: " + beta_0 + ", dxx: " + dxx + ", dyy: " + dyy + ", T2: " + T2);
			log.debug("bixx: " + bixx + ", biyy: " + biyy);

			double limFunc = 300;
			if (T2 > 1 && (bixx < limFunc))
			{
				double rho_0_lim = (limFunc + xRobot*cos_th + yRobot*sin_th)/(sin_beta_0*sin_th + cos_beta_0*cos_th);
				double dxx_lim = rho_0_lim*cos_beta_0 - xRobot, dyy_lim = rho_0_lim*sin_beta_0 - yRobot;
				double biyy_lim = (dyy_lim*cos_th - dxx_lim*sin_th), bixx_lim = (dxx_lim*cos_th + dyy_lim*sin_th);
				double u_i_lim = uCenter - f*biyy_lim/bixx_lim;
				double u_max = 2*u_i_lim;
				log.debug("rho_0_lim: " + rho_0_lim + ", dxx_lim: " + dxx_lim + ", dyy_lim: " + dyy_lim + ", bixx_lim: " + bixx_lim + ", biyy_lim: " + biyy_lim);

				double rho_0_new = 2*rho_0_lim - rho_0;
				double dxx_new = rho_0_new*cos_beta_0 - xRobot, dyy_new = rho_0_new*sin_beta_0 - yRobot;
				double biyy_new = (dyy_new*cos_th - dxx_new*sin_th), bixx_new = (dxx_new*cos_th + dyy_new*sin_th);
				double u_new = uCenter - f*biyy_new/bixx_new;
				log.debug("rho_0_new: " + rho_0_new + ", dxx_new: " + dxx_new + ", dyy_new: " + dyy_new + ", bixx_new: " + bixx_new + ", biyy_new: " + biyy_new);

				log.debug("u_i_lim: " + u_i_lim + ", u_max: " + u_max + ", u_new: " + u_new);
				obsPredicted.setComponent(lIdx, (u_max - u_new));
			}
			else
				obsPredicted.setComponent(lIdx, (uCenter - f*biyy/bixx));
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
		AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(lineMap.getNumberOfStateLines());

		Iterator itLines = lineMap.getStateLineIndexIterator();
		int countLineIdx = 0;
		while (itLines.hasNext())
		{
			VLine line = lineMap.nextLine(itLines);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);

			covar.setElement(countLineIdx, countLineIdx, lineProj.getUErrorSigma());
			countLineIdx++;
		}

		return covar;
//		return MatrixUtil.clone(sensorCovariance);
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
		AbstractDoubleMatrix H = new DoubleMatrix(lineMap.getNumberOfStateLines(), state.dimension());

		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();

		int lIdx = 0;
		Iterator itLines = lineMap.getStateLineIndexIterator();
		for (; lIdx < lineMap.getNumberOfStateLines(); lIdx++)
		{
			VLine line = lineMap.nextLineModel(itLines);
//			VLine line = lineMap.nextLine(itLines);
			RigidBodyTranformation2D rbTrafoIni = lineMap.getLineModelRigidBodyTranformation2D(line);
			log.debug("line: " + line + ", rbTrafoIni: " + rbTrafoIni);

			double [] arrayLineTrafo = new double[2];
			rbTrafoIni.inverseLine(state.getComponent(0), state.getComponent(1), arrayLineTrafo);

			double dX = arrayLineTrafo[0], dY = arrayLineTrafo[1];
			double thetaRobot = state.getComponent(2) - rbTrafoIni.getAsPose().getTheta();
			double sinTh = Math.sin(thetaRobot), cosTh = Math.cos(thetaRobot);


			double rho_0_lin = state.getComponent(3 + lIdx*2);
			double beta_0 = state.getComponent(3 + lIdx*2 + 1);
			double sin_beta_0 = Math.sin(beta_0);
			double cos_beta_0 = Math.cos(beta_0);

			double V = cosTh*(rho_0_lin*sin_beta_0 - dY/f) - sinTh*(rho_0_lin*cos_beta_0 - dX/f);
			double Z = cosTh*(rho_0_lin*cos_beta_0 - dX/f) + sinTh*(rho_0_lin*sin_beta_0 - dY/f);
			double Z2 = Z*Z;

			if (Z2 > 0.01)
			{
				// Menos na frente de todos devido a subtração do uCenter ...
				H.setElement(lIdx, 0, -(sinTh*Z + cosTh*V)/Z2);
				H.setElement(lIdx, 1, -(-cosTh*Z + sinTh*V)/Z2); 
				H.setElement(lIdx, 2, f*(1 + V*V/Z2));
				H.setElement(lIdx, 3 + lIdx*2, -f*((cosTh*sin_beta_0 - sinTh*cos_beta_0)*Z - (cosTh*cos_beta_0 + sinTh*sin_beta_0)*V)/Z2);
				H.setElement(lIdx, 3 + lIdx*2 + 1, -f*((cosTh*rho_0_lin*cos_beta_0 + sinTh*rho_0_lin*sin_beta_0)*Z + 
													 (cosTh*rho_0_lin*sin_beta_0 - sinTh*rho_0_lin*cos_beta_0)*V)/Z2);

////				H.setElement(lIdx, 3 + lIdx*2, -((cosTh*sin_beta_0 - sinTh*cos_beta_0)*Z + (cosTh*cos_beta_0 + sinTh*sin_beta_0)*V)/Z2);
////				H.setElement(lIdx, 3 + lIdx*2 + 1, -((cosTh*rho_0_lin*cos_beta_0 + sinTh*rho_0_lin*sin_beta_0)*Z + (cosTh*rho_0_lin*sin_beta_0 - sinTh*rho_0_lin*cos_beta_0)*V)/Z2);
			}
			else
			{
				log.debug("Z == 0");
			}
			log.debug("H: \r\n" + MatrixUtil.toString(H, 9, 6));
		}
		return H;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction
/*
	sin_beta_0 = sin(beta_0(i));
	cos_beta_0 = cos(beta_0(i));
	rho_0_abs = rho_0(i);

	dxx = rho_0_abs*cos_beta_0 - xRobot;
	T2 = xRobot*xRobot + yRobot*yRobot;
	dyy = rho_0_abs*sin_beta_0 - yRobot;
	rho_i(i) = sqrt(dxx*dxx + dyy*dyy);
	sin_th = sin(th_r(i));
	cos_th = cos(th_r(i));
	biyy(i) = (dyy*cos_th - dxx*sin_th);
	bixx(i) = (dxx*cos_th + dyy*sin_th);
	sin_bi = biyy(i)/rho_i(i);
	cos_bi = bixx(i)/rho_i(i);
	sin_bi_a(i) = sin_bi;
	cos_bi_a(i) = cos_bi;
	beta_i(i)  = asin(dyy/rho_i(i));
	gamma_i(i) = asin(sin_bi);
	u_est(i) = uCenter - f*biyy(i)/bixx(i);

	if T2 > 1 && (bixx(i) < 300)
		sinGamma0 = (sin_beta_0*cos_th - cos_beta_0*sin_th);
		cosGamma0 = (cos_beta_0*cos_th + sin_beta_0*sin_th);
		u_inf = uCenter - f*sinGamma0/cosGamma0;
		u_i_inf(i) = u_inf;

        rho_0_lim(i) = (300 + xRobot*cos_th + yRobot*sin_th)/(sin_beta_0*sin_th + cos_beta_0*cos_th);
   		dxx_lim = rho_0_lim(i)*cos_beta_0 - xRobot;
   		dyy_lim = rho_0_lim(i)*sin_beta_0 - yRobot;
		rho_i_lim(i) = sqrt(dxx_lim*dxx_lim + dyy_lim*dyy_lim);
		biyy_lim = (dyy_lim*cos_th - dxx_lim*sin_th);
		bixx_lim = (dxx_lim*cos_th + dyy_lim*sin_th);
        u_i_lim(i) = uCenter - f*biyy_lim/bixx_lim;

   		dxx_lim = rho_0_lim(i)*cos_beta_0 - xRobot;
   		dyy_lim = rho_0_lim(i)*sin_beta_0 - yRobot;
		rho_i_lim(i) = sqrt(dxx_lim*dxx_lim + dyy_lim*dyy_lim);
        
		u_max = 2*u_i_lim(i);% - 0*u_inf;
        u_i_max(i) = u_max;

   		rho_0_new = 2*rho_0_lim(i) - rho_0(i);
		rho_0_corr(i) = rho_0_new;

		dxx_new = rho_0_new*cos_beta_0 - xRobot;
		dyy_new = rho_0_new*sin_beta_0 - yRobot;
		rho_i_corr(i) = sqrt(dxx_new*dxx_new + dyy_new*dyy_new);
		biyy_new = (dyy_new*cos_th - dxx_new*sin_th);
		bixx_new = (dxx_new*cos_th + dyy_new*sin_th);

		u_new = uCenter - f*biyy_new/bixx_new;
		u_est_corr(i) = u_max - u_new;
	else
		u_est_corr(i) = u_est(i);
	end;
//*/


	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(getDataDimension(), sigmaIn.columns()); 
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			double xRobotActual = sigmaIn.getElement(0, idxInput), yRobotActual = sigmaIn.getElement(1, idxInput);
			double thetaRobotActual = sigmaIn.getElement(2, idxInput);
			log.debug("xRobotActual: " + xRobotActual + ", yRobotActual: " + yRobotActual + ", thetaRobotActual: " + thetaRobotActual);

			Iterator itLines = lineMap.getStateLineIndexIterator();
			for (int lIdx = 0; lIdx < lineMap.getNumberOfStateLines(); lIdx++)
			{
				VLine line = lineMap.nextLineModel(itLines);
				RigidBodyTranformation2D rbTrafoIni = lineMap.getLineModelRigidBodyTranformation2D(line);
				log.debug("line: " + line + ", rbTrafoIni: " + rbTrafoIni);

				double [] arrayLineTrafo = new double[2];
				rbTrafoIni.inverseLine(xRobotActual, yRobotActual, arrayLineTrafo);

				double xRobot = arrayLineTrafo[0], yRobot = arrayLineTrafo[1];
				double thetaRobot = thetaRobotActual - rbTrafoIni.getAsPose().getTheta();
				double sin_th = Math.sin(thetaRobot), cos_th = Math.cos(thetaRobot);

				double rho_0_lin = sigmaIn.getElement(3 + lIdx*2, idxInput);
				double beta_0 = sigmaIn.getElement(3 + lIdx*2 + 1, idxInput);
				double rho_0 = f * rho_0_lin;
				double sin_beta_0 = Math.sin(beta_0), cos_beta_0 = Math.cos(beta_0);

				double dxx = rho_0*cos_beta_0 - xRobot, dyy = rho_0*sin_beta_0 - yRobot;
				double T2 = xRobot*xRobot + yRobot*yRobot;
				double biyy = (dyy*cos_th - dxx*sin_th), bixx = (dxx*cos_th + dyy*sin_th);
				log.debug("rho_0: " + rho_0 + ", beta_0: " + beta_0 + ", dxx: " + dxx + ", dyy: " + dyy + ", T2: " + T2);
				log.debug("bixx: " + bixx + ", biyy: " + biyy);

				double limFunc = 300;
				if (T2 > 1 && (bixx < limFunc))
				{
					double rho_0_lim = (limFunc + xRobot*cos_th + yRobot*sin_th)/(sin_beta_0*sin_th + cos_beta_0*cos_th);
					double dxx_lim = rho_0_lim*cos_beta_0 - xRobot, dyy_lim = rho_0_lim*sin_beta_0 - yRobot;
					double biyy_lim = (dyy_lim*cos_th - dxx_lim*sin_th), bixx_lim = (dxx_lim*cos_th + dyy_lim*sin_th);
					double u_i_lim = uCenter - f*biyy_lim/bixx_lim;
					double u_max = 2*u_i_lim;
					log.debug("rho_0_lim: " + rho_0_lim + ", dxx_lim: " + dxx_lim + ", dyy_lim: " + dyy_lim + ", bixx_lim: " + bixx_lim + ", biyy_lim: " + biyy_lim);

					double rho_0_new = 2*rho_0_lim - rho_0;
					double dxx_new = rho_0_new*cos_beta_0 - xRobot, dyy_new = rho_0_new*sin_beta_0 - yRobot;
					double biyy_new = (dyy_new*cos_th - dxx_new*sin_th), bixx_new = (dxx_new*cos_th + dyy_new*sin_th);
					double u_new = uCenter - f*biyy_new/bixx_new;
					log.debug("rho_0_new: " + rho_0_new + ", dxx_new: " + dxx_new + ", dyy_new: " + dyy_new + ", bixx_new: " + bixx_new + ", biyy_new: " + biyy_new);

					log.debug("u_i_lim: " + u_i_lim + ", u_max: " + u_max + ", u_new: " + u_new);

					sigmaOut.setElement(lIdx, idxInput, (u_max - u_new) + sigmaError.getElement(lIdx, idxInput));
				}
				else
				{
					sigmaOut.setElement(lIdx, idxInput, (uCenter - f*biyy/bixx) + sigmaError.getElement(lIdx, idxInput));
				}
			}
		}
		return sigmaOut;
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
	}
}

