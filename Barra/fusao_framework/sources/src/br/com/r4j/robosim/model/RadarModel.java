package br.com.r4j.robosim.model;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.robosim.MapDependent;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.robosim.simrobot.SimRadarSensor;


/** @modelguid {DBC35192-F5DB-4ED9-B5E2-98427F03D006} */
public class RadarModel implements SensorModel, Configurable, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, 	PosteriorProbabilityDensityFunction, InvertibleEKFDoubleVectorFunction, MapDependent
{
	/** @modelguid {6FDA352D-9AC8-4E9F-92B6-FAA645079D88} */
	private static Log log = LogFactory.getLog(RadarModel.class.getName());
	/** @modelguid {F4666DD1-33C5-461F-84D1-807C984EC35C} */
	private static Log logSens = LogFactory.getLog("radar");

	/** @modelguid {35463848-B291-4A68-97F8-6A3C5A547CEA} */
	private SimRadarSensor sns = null;
	/** @modelguid {AAC4844B-BC5F-4ED1-824D-9A838FFE4B8C} */
	private WorldMap map = null;

	/** @modelguid {1ACDB019-D69B-4A6C-B3D4-5FA8B9F6262B} */
	private Random rnd = null;


	/** @modelguid {B949789F-13BF-4EE6-AB87-473C0D5A9770} */
	public RadarModel()
	{
		rnd = new Random((new Date()).getTime());
	}


	/** @modelguid {91988BC2-89B7-40DE-9294-C9E6580869D7} */
	public void setSensor(Sensor sns)
	{
		this.sns = (SimRadarSensor) sns;
	}


	/** @modelguid {40121F1A-4EE7-4AA6-8A9D-B9DF24DE7215} */
	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


	/** @modelguid {2B80AEC7-999F-4D86-B9BC-280AA56D8DC9} */
	public String getName()
	{
		return "Radar Model";
	}

	
	/** @modelguid {F620C99C-257C-4252-A625-235F0B295190} */
	public void configure(PropertiesHolder props, String strBaseKey)
	{
	}


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 * @modelguid {4D03AB2A-B4D8-4966-B98B-BA7075DF974F}
	 */
	public void dataAvailable()
	{
		// não precisa fazer nada ...
	}


	/** @modelguid {7AA3DE3E-320D-4865-97EE-EFCE84132972} */
	public int getDataDimension()
	{
		return 3;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 * @modelguid {A4778CC4-14E0-4EA4-9E02-012EF7D6C1CA}
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector state, AbstractDoubleVector snsReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector obsPredicted = new DoubleVector(getDataDimension());
		this.produceResults(state, obsPredicted);
		return obsPredicted;
	}


	/** @modelguid {7099F4DE-C186-4323-9CF5-5E2EB8531BCE} */
	protected void produceResults(AbstractDoubleVector state, AbstractDoubleVector obsPredicted)
	{
		double dX = state.getComponent(0) - sns.getRadarPose().getX();
		double dY = state.getComponent(1) - sns.getRadarPose().getY();

		if (dX > 0)
			obsPredicted.setComponent(0, Math.sqrt(dX*dX + dY*dY));
		else
			obsPredicted.setComponent(0, -Math.sqrt(dX*dX + dY*dY));

		if (dX*dX > 0.0001)
			if (dX > 0)
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.atan(dY/dX), sns.getLastRads()));
			else
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.atan(dY/dX) + Math.PI, sns.getLastRads()));
		else
			if (dY > 0)
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.PI/2, sns.getLastRads()));
			else
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(3*Math.PI/2, sns.getLastRads()));

		obsPredicted.setComponent(2, state.getComponent(2));
	}


	/** @modelguid {39CCF1BE-8243-4BB6-8A9A-1447D749E6A4} */
	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance)
	{
		return MatrixUtil.clone(sensorCovariance);
	}


	/** @modelguid {96A191AB-8693-4D79-89C8-51AFF202F743} */
	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings)
	{
		return sensorReadings;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface EKFDoubleVectorFunction

	/** @modelguid {05A47D85-8FE2-4D8A-9334-82029BBC3DFF} */
	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		AbstractDoubleMatrix H = new DoubleMatrix(3, 3);
		double dX = state.getComponent(0) - sns.getRadarPose().getX();
		double dY = state.getComponent(1) - sns.getRadarPose().getY();

		double e11 = 1, e12 = 1, e21 = 1, e22 = 1;

		double base2 = 1 + (dY*dY)/(dX*dX);
		e22 = 1/(dX*base2);
		e21 = -(dY/dX)*e22;

		double base1 = Math.sqrt(dX*dX + dY*dY);
		e11 = (dX/Math.abs(dX))*dX/base1;
		e12 = (dX/Math.abs(dX))*dY/base1;

		H.setElement(0, 0, e11); H.setElement(0, 1, e12); H.setElement(0, 2, 0);
		H.setElement(1, 0, e21); H.setElement(1, 1, e22); H.setElement(1, 2, 0);
		H.setElement(2, 0, 0); H.setElement(2, 1, 0); H.setElement(2, 2, 1);

		return H;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	/** @modelguid {43D3F0B7-8E40-4965-B843-E411CC779B41} */
	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(sigmaIn.rows(), sigmaIn.columns()); 
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			double dX = sigmaIn.getElement(0, idxInput) - sns.getRadarPose().getX();
			double dY = sigmaIn.getElement(1, idxInput) - sns.getRadarPose().getY();
			
			if (dX > 0)
				sigmaOut.setElement(0, idxInput, Math.sqrt(dX*dX + dY*dY) + sigmaError.getElement(0, idxInput));
			else
				sigmaOut.setElement(0, idxInput, -Math.sqrt(dX*dX + dY*dY) + sigmaError.getElement(0, idxInput));
				
			if (dX*dX > 0.0001)
				if (dX > 0)
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.atan(dY/dX) + sigmaError.getElement(1, idxInput), sns.getLastRads()));
				else
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.atan(dY/dX) + Math.PI + sigmaError.getElement(1, idxInput), sns.getLastRads()));
			else
				if (dY > 0)
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.PI/2 + sigmaError.getElement(1, idxInput), sns.getLastRads()));
				else
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(3*Math.PI/2 + sigmaError.getElement(1, idxInput), sns.getLastRads()));

			sigmaOut.setElement(2, idxInput, sigmaIn.getElement(2, idxInput) + sigmaError.getElement(2, idxInput));
		}
		return sigmaOut;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface InvertibleEKFDoubleVectorFunction

	/** @modelguid {50D3E844-0265-4C18-B13E-9B1EB30506EA} */
	public AbstractDoubleVector produceInverseResults(AbstractDoubleVector stateEstimatei_minus1, AbstractDoubleVector readings)
	{
		AbstractDoubleVector output = new DoubleVector(3);

		double r = readings.getComponent(0);
		double theta = readings.getComponent(1);

		if (r < 0)
		{
			theta += Math.PI;
			r = -r;
		}

		output.setComponent(0, r*Math.cos(theta) + sns.getRadarPose().getX());
		output.setComponent(1, r*Math.sin(theta) + sns.getRadarPose().getY());
		output.setComponent(2, readings.getComponent(2));
		
		return output;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction#getInverseTransitionMatrixJacobian(JSci.maths.AbstractDoubleVector)
	 * @modelguid {EFA486DD-87E2-43A2-A7D3-11C16D0F3C14}
	 */
	public AbstractDoubleMatrix getInverseTransitionMatrixJacobian(AbstractDoubleVector readings)
	{
		AbstractDoubleMatrix H = new DoubleMatrix(3, 3);

		double r = readings.getComponent(0);
		double theta = readings.getComponent(1);

		if (r < 0)
		{
			theta += Math.PI;
			r = -r;
		}

		double e11 = Math.cos(theta);
		double e21 = Math.sin(theta);

		double e12 = -1*r*Math.sin(theta);
		double e22 = r*Math.cos(theta);
		H.setElement(0, 0, e11); H.setElement(0, 1, e12); H.setElement(0, 2, 0);
		H.setElement(1, 0, e21); H.setElement(1, 1, e22); H.setElement(1, 2, 0);
		H.setElement(2, 0, 0); H.setElement(2, 1, 0); H.setElement(2, 2, 1);
		return H;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface PosteriorProbabilityDensityFunction

	/** @modelguid {63E310F8-EBE6-4459-8921-262394C397CC} */
	private AbstractDoubleSquareMatrix covar = null;
	/** @modelguid {5608D654-CD8D-4AC7-A3EA-BB57C72598FC} */
	private AbstractDoubleSquareMatrix covarInv = null;
	/** @modelguid {B0A7A26E-77A5-4AEA-971B-22247D0A8FA4} */
	private AbstractDoubleVector readings = null;
	/** @modelguid {D03C5CD0-E394-423A-B23D-0FAC3890DEED} */
	private double pdfConst = 0;

	/** @modelguid {D9D14C4A-473C-4748-ADD2-8DA9FD67E724} */
	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar)
	{
		this.covar = covar;
		covarInv = covar.inverse();
		pdfConst = 1.0/Math.sqrt(((AbstractDoubleSquareMatrix) covar.scalarMultiply(2*Math.PI)).det());
		if (log.isDebugEnabled())
		{
			log.debug("covar: \r\n" + MatrixUtil.toString(covar, 9, 4));
			log.debug("covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
			log.debug("pdfConst: " + pdfConst);
		}
		if (pdfConst < 1)
			pdfConst = 1;
		if (log.isDebugEnabled())
			log.debug("pdfConst(2): " + pdfConst);
	}


	/** @modelguid {7B47EDCA-C144-42AD-B4B2-62F9D32B54F1} */
	public void copyParticleData(int j, int i)
	{
	}


	/** @modelguid {7A05C90B-53AF-41E5-B190-D8799854CF38} */
	public void setObservations(AbstractDoubleVector readings)
	{
		this.readings = readings;
	}


	/** @modelguid {A0EE709E-E7F2-4C62-9C4D-79110709961D} */
	public double stateProbability(AbstractDoubleMatrix input, int rowObjectCount)
	{
		DoubleVector vectVal = new DoubleVector(readings.dimension());
		DoubleVector inputVectVal = new DoubleVector(input.columns());
		for (int i = 0; i < input.columns(); i++)
			inputVectVal.setComponent(i, input.getElement(rowObjectCount, i));

		this.produceResults(inputVectVal, vectVal);
//		log.debug("inputVectVal: \r\n" + MatrixUtil.toString(inputVectVal, 9, 4));
//		log.debug("vectVal: \r\n" + MatrixUtil.toString(vectVal, 9, 4));
//		log.debug("readings: \r\n" + MatrixUtil.toString(readings, 9, 4));
		AbstractDoubleVector diff = vectVal.subtract(readings);
//		log.debug("diff: \r\n" + MatrixUtil.toString(diff, 9, 4));
		double expoent = covarInv.multiply(diff).scalarProduct(diff);
//		logPF.debug("covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
//		logPF.debug("covarInv.multiply(diff): \r\n" + MatrixUtil.toString(covarInv.multiply(diff), 9, 4));
//		log.debug("expoent: " + expoent);
//		return pdfConst*Math.exp(-1.0/2.0*expoent + 20);
		return pdfConst*Math.exp(-1.0/2.0*expoent);
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction#canProduceObservations(JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix, JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix)
	 */
	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		return true;
	}

	public boolean canProduceObservationsPF(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		return true;
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
	}
}

