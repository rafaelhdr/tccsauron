package br.com.r4j.math.kalmanfilter.test;

import java.io.File;
import java.io.IOException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PlainConfigurator;
import br.com.r4j.math.kalmanfilter.AditiveNosieUnscentedKalmanFilter;
import br.com.r4j.math.kalmanfilter.DoubleVectorFunction;
import br.com.r4j.robosim.Pose2D;


/** 
 * 
 */
public class SimpleRobotWalkerAndOneBeamDetectorStateEstimator
{
	private static Log log = LogFactory.getLog(SimpleRobotWalkerAndOneBeamDetectorStateEstimator.class.getName());
	private static Log log_ukf_res1 = LogFactory.getLog("ukf_res1");

	private Pose2D poseRobotIniEst = null;
	private DoubleSquareMatrix pRobotMove = null;

	private Pose2D poseRobotIniReal = null;
	private Pose2D poseRobotLast = null;

	private Pose2D poseRadar = null;
	private DoubleSquareMatrix pRadarMeasure = null;
	
	private OneFixedRadarDoubleVectorFunction radarMeasureFunction = null;
	private Pose2DMoveStateDoubleVectorFunction moveStateFunction = null;
	private AditiveNosieUnscentedKalmanFilter ukf = null;


	public SimpleRobotWalkerAndOneBeamDetectorStateEstimator()
	{
	}

	public void initStates()
	{
		poseRadar = new Pose2D(0, 0, 0);
		pRadarMeasure = new DoubleSquareMatrix(2);
/*
		pRadarMeasure.setElement(0, 0, 50*50*1.000000);
		pRadarMeasure.setElement(1, 1, (5*Math.PI/180)*(5*Math.PI/180)*1.000000);
/*/
		pRadarMeasure.setElement(0, 0, 1.0*1.0);
		pRadarMeasure.setElement(1, 1, (0.02*Math.PI/180)*(0.02*Math.PI/180));
//*/

		radarMeasureFunction = new OneFixedRadarDoubleVectorFunction();
		radarMeasureFunction.setRadarPose(poseRadar);
		radarMeasureFunction.setRadarPoseCovar(pRadarMeasure);

//		poseRobotIniReal = new Pose2D(50000, 10000, 165*Math.PI/180);
		poseRobotIniReal = new Pose2D(750, -500, 0*Math.PI/180);
		poseRobotLast = poseRobotIniReal;
		pRobotMove = new DoubleSquareMatrix(2);
/*
		pRobotMove.setElement(0, 0, 1000.0*1000.0); // por metro.
		pRobotMove.setElement(1, 1, 500.0*500.0); // por metro.
/*/
		pRobotMove.setElement(0, 0, 50.0*50.0); // por metro.
		pRobotMove.setElement(1, 1, 50.0*50.0); // por metro.
//*/
//		pRobotMove.setElement(2, 2, (1*Math.PI/180)*(1*Math.PI/180));

		moveStateFunction = new Pose2DMoveStateDoubleVectorFunction();
		moveStateFunction.setRobotMoveCovar(pRobotMove);

		ukf = new AditiveNosieUnscentedKalmanFilter();
		ukf.setStateTransitionFunction((DoubleVectorFunction) moveStateFunction);
		ukf.setMeasureEstimationFunction((DoubleVectorFunction) radarMeasureFunction);
		
//		poseRobotIniEst = new Pose2D(50000 - 30000, 10000 - 5000, 160*Math.PI/180);
		poseRobotIniEst = new Pose2D(50000 - 300, 10000 - 50, 160*Math.PI/180);
		DoubleSquareMatrix pRobotIniEst = new DoubleSquareMatrix(2);
//		pRobotIniEst.setElement(0, 0, 40000*40000);
//		pRobotIniEst.setElement(1, 1, 10000*10000);
		pRobotIniEst.setElement(0, 0, 400*400);
		pRobotIniEst.setElement(1, 1, 100*100);
		log.debug("pXX: \r\n" + MatrixUtil.toString(pRobotIniEst, 12, 4));
//		pRobotIniEst.setElement(2, 2, (20*Math.PI/180)*(20*Math.PI/180));
		DoubleVector vectIni = new DoubleVector(2);
		vectIni.setComponent(0, poseRobotIniEst.getX());
		vectIni.setComponent(1, poseRobotIniEst.getY());
//		vectIni.setComponent(2, poseRobotIniEst.getTheta());
		ukf.setLastState(vectIni, pRobotIniEst, null);
	}


	public void updateEstimates(double d, double dTheta)
	{
		radarMeasureFunction.setStates(d, dTheta);
		moveStateFunction.setStates(d, dTheta);

		// Monta o vetor de medição e a matriz de erro.
		poseRobotLast = moveStateFunction.getPose(poseRobotLast, d, dTheta);
		AbstractDoubleVector measures = radarMeasureFunction.getTrueMeasure(poseRobotLast);

		// Aplica o UKF.
		ukf.setMeasure(measures, pRadarMeasure);
//		ukf.setMeasure(null, null);
		ukf.setStateTransitionErrorInc(moveStateFunction.getStateTransitionErrorInc());

		ukf.update();
	}


	public void runEngine()
	{
/*
		double d = 100;
		double theta = 5*Math.PI/180;
/*/
		double d = 1000;
		double theta = 0;
//*/

		int count = 0;
		while (++count < 1000)
		{
			long start_t = System.currentTimeMillis();

			this.updateEstimates(d, theta);

			AbstractDoubleVector trueMeas = radarMeasureFunction.getTrueMeasure(poseRobotLast);
			AbstractDoubleVector vetResults = ukf.getEstimateExpectancy();
			AbstractDoubleMatrix covResults = ukf.getEstimateCovariance();
			Pose2D poseEst = new Pose2D(vetResults.getComponent(0), vetResults.getComponent(1), 0);
			log.debug("pose real     = " + poseRobotLast);
			log.debug("pose estimado = " + poseEst);
			double diffTot = Math.sqrt((poseEst.getX() - poseRobotLast.getX())*(poseEst.getX() - poseRobotLast.getX()) +
			                           (poseEst.getY() - poseRobotLast.getY())*(poseEst.getY() - poseRobotLast.getY()));
			double diffTotExp = Math.sqrt(covResults.getElement(0, 0) + covResults.getElement(1, 1));
			log_ukf_res1.debug("difX, diffY, diffTot, diffTotExp: (" + (poseEst.getX() - poseRobotLast.getX()) + ", " + (poseEst.getY() - poseRobotLast.getY()) + ", " + diffTot + ", " + diffTotExp + "), " + "meas: (" + trueMeas.getComponent(0) + ", " + trueMeas.getComponent(1) + ")");

			theta += 360.0/8*Math.PI/180;
//			theta += 10*Math.PI/180*100/(100 + count);
		}
    }

	
	public static void main(String [] args)
	{
		try
		{
			if (args.length > 0)
				PlainConfigurator.createConfigurator(new File(args[0]));
			else
				PlainConfigurator.createConfigurator(new File("conf/conf.xml"));
		}
		catch (ConfiguratorException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}

		SimpleRobotWalkerAndOneBeamDetectorStateEstimator estimator = new SimpleRobotWalkerAndOneBeamDetectorStateEstimator();
		estimator.initStates();
		estimator.runEngine();
	}
}


/*

1362 - pose real = (56202.90189072868,-24055.648934332086,1594.2535519417047)
1362 - pose estimado = 055,688.056; -024,690.349; 


1392 - pose real = (56202.90189072868,-24055.648934332086,1594.2535519417047)
1392 - pose estimado = 059,612.394; -016,926.588; 


881  - pose real = (56202.90189072868,-24055.648934332086,1594.2535519417047)
881  - pose estimado = 055,766.953; -023,205.917; 


1653 - pose real = (33560.274014848444,-19250.30579816306,8121.943227118719)
1653 - pose estimado = 027,809.800; -023,583.010; 

1272 - pose real = (33560.274014848444,-19250.30579816306,8121.943227118719)
1272 - pose estimado = 026,863.809; -024,518.681; 

2083 - pose real = (33560.274014848444,-19250.30579816306,8121.943227118719)
2083 - pose estimado = 034,257.077; -017,901.780; 

//*/







