package br.com.r4j.research.image.sequence.featurematch.vline.test;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PlainConfigurator;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.SpatialFilterOneInputImageOp;
import br.com.r4j.jmr.datasource.RobotDescription;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.image.sequence.featurematch.vline.Matcher;
import br.com.r4j.research.image.sequence.sfm.VLineMatcherWrapperImageGrabber;
import br.com.r4j.research.pose2destimation.LogFilePose2DGrabber;
import br.com.r4j.research.pose2destimation.Pose2DGrabber;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;


/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 */
public class VLineMatcherRunner implements Pose2DGrabber
{
	private static Log log = LogFactory.getLog(VLineMatcherRunner.class.getName());
	private static Log log_ukf_res1 = LogFactory.getLog("ukf_res1");
	private static Log logTime = LogFactory.getLog("time");

	private Matcher matcher = null;
	private LogFilePose2DGrabber poseGrabber = null;
	private VLineMatcherWrapperImageGrabber imgGrabber = null; 
	private RobotDescription robotDesc = null;

	private File flDirImgs = null;
	private int [] outData = null;

	private Pose2D poseLast = null;

	private double minDistBetweenFrames;
	private double minAngleBetweenFrames;
	private int frames2analyze;


	public VLineMatcherRunner()
	{
		matcher = new Matcher();
		poseGrabber = new LogFilePose2DGrabber();
		imgGrabber = new VLineMatcherWrapperImageGrabber();
	}


	public Pose2D getPose(long imageTimestamp)
	{
		return poseGrabber.getPose(imageTimestamp);
	}


	public Pose2D getMovement(long imageTimestamp)
	{
		if (poseLast == null)
			return new Pose2D(0, 0, 0);
		else
		{
			Pose2D pose = poseGrabber.getPose(imageTimestamp);
			Pose2D poseMove = pose.sub(poseLast);
			double cosThetaAnt = Math.cos(poseLast.getTheta()), sinThetaAnt = Math.sin(poseLast.getTheta());
			double dX = poseMove.getX();
			double dY = poseMove.getY();
			double dXRot = cosThetaAnt*dX + sinThetaAnt*dY;
			double dYRot = cosThetaAnt*dY - sinThetaAnt*dX;
			double dTheta = poseMove.getTheta();
			poseMove = new Pose2D(dXRot, dYRot, dTheta);
			return poseMove;
		}
	}


	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp)
	{
		return null;
	}


	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp)
	{
		DoubleSquareMatrix stateCov = new DoubleSquareMatrix(3);
		if (poseLast != null)
		{
			Pose2D pose = poseGrabber.getPose(imageTimestamp);
			Pose2D poseMove = pose.sub(poseLast);
			double cosThetaAnt = Math.cos(poseLast.getTheta()), sinThetaAnt = Math.sin(poseLast.getTheta());
			double dX = poseMove.getX();
			double dY = poseMove.getY();
			double dXRot = cosThetaAnt*dX + sinThetaAnt*dY;
			double dYRot = cosThetaAnt*dY - sinThetaAnt*dX;
			double dTheta = poseMove.getTheta();
			stateCov.setElement(0, 0, robotDesc.getDXVariancePerMilimiter()*dXRot*dXRot);
			stateCov.setElement(1, 1, robotDesc.getDYVariancePerMilimiter()*dYRot*dYRot);
			stateCov.setElement(2, 2, robotDesc.getDThetaVariancePerMillimiter()*dTheta*dTheta);
		}
		return stateCov;
	}


	public boolean initEngine() throws IOException
	{
		PropertiesHolder props = Configurator.getPropsHolder();

		flDirImgs = new File(props.getStringProperty("est-img-seq/imgs-dir-name"));
		if (!flDirImgs.exists())
		{
			log.debug(flDirImgs.getPath() + " não exsite!");
			return false;
		}

		CameraModel camModel = new CameraModel();
		WorldMap worldMap = new WorldMap();
		robotDesc = new RobotDescription();

		worldMap.setMapFile(props.getFileProperty("est-img-seq/worldmap"));

		Pose2D pose = new Pose2D(props.getIntegerProperty("est-img-seq/shift-x").intValue(), props.getIntegerProperty("est-img-seq/shift-y").intValue(), props.getDoubleProperty("est-img-seq/shift-theta").doubleValue()*Math.PI/180);
		Pose2D poseCam = new Pose2D(props.getIntegerProperty("est-img-seq/cam-x").intValue(), props.getIntegerProperty("est-img-seq/cam-y").intValue(), props.getDoubleProperty("est-img-seq/cam-theta").doubleValue()*Math.PI/180);

		poseGrabber.setDataFile(props.getStringProperty("est-img-seq/data-file-name"));
		poseGrabber.setPoseOffset(pose.add(poseCam));
		poseGrabber.setRobotDescription(robotDesc);
		poseGrabber.setInitalState(pose.add(poseCam));

		imgGrabber.setMatcher(matcher);
		imgGrabber.setPose2DGrabber(this);

		matcher.setCameraModel(camModel);
		matcher.setWorldMap(worldMap);
		matcher.setUseWindow(props.getBooleanProperty("est-img-seq/use-window").booleanValue());
		matcher.setUseDirectionWindow(props.getBooleanProperty("est-img-seq/use-direct-window").booleanValue());
		matcher.setDirectWindowSize(props.getIntegerProperty("est-img-seq/direct-window-size").intValue());
		matcher.setMinDepth(props.getIntegerProperty("est-img-seq/min-depth").intValue());
		matcher.setHashVectorTreshold(props.getDoubleProperty("est-img-seq/hash-treshold").doubleValue());
		matcher.setHashVectorSideBandSize(props.getIntegerProperty("est-img-seq/hash-side-band").intValue());
		matcher.setHashVectorAttenuationMax(props.getDoubleProperty("est-img-seq/hash-attenuation").doubleValue());
		matcher.setHashVectorHistorySize(props.getIntegerProperty("est-img-seq/hash-hist-size").intValue());
		matcher.setCompensateIllumination(props.getBooleanProperty("est-img-seq/compensate-illumination").booleanValue());
		matcher.setSimpleMatch(props.getBooleanProperty("est-img-seq/simple-match").booleanValue());

		camModel.setFocusDistance(props.getIntegerProperty("est-img-seq/focal-distance").intValue());
		camModel.setUAxisPixelCenter(props.getIntegerProperty("est-img-seq/u-center").intValue());
		camModel.setVAxisPixelCenter(props.getIntegerProperty("est-img-seq/v-center").intValue());
		camModel.setUAxisPixelCount(props.getIntegerProperty("est-img-seq/u-count").intValue());
		camModel.setVAxisPixelCount(props.getIntegerProperty("est-img-seq/v-count").intValue());

		poseGrabber.initFileParser();

		minDistBetweenFrames = props.getDoubleProperty("est-img-seq/frame-selection/min-dist").doubleValue();
		minAngleBetweenFrames = props.getDoubleProperty("est-img-seq/frame-selection/min-angle").doubleValue();

		frames2analyze = props.getIntegerProperty("est-img-seq/frames-to-analyze").intValue();

		// Base time calc.
		{
			try
			{
				File flImg = br.com.r4j.commons.file.FileSystem.getFirstFile(flDirImgs);

				BufferedImage buffImg = ImageUtil.getImageBMP(flImg);
				int [] inDataTmp = ImageUtil.getThreeBandPackedData(buffImg);
				int [] tmpData = new int[inDataTmp.length];
				SpatialFilterOneInputImageOp closingOp = new SpatialFilterOneInputImageOp();
				closingOp.setFilter(new float [] {0, 1, 0,
												1, 1, 1,
												0, 1, 0},
										3, 3);
				long tmTm = System.currentTimeMillis();
				for (int i = 0; i < 100; i++)
						closingOp.operate(inDataTmp, tmpData, buffImg.getWidth(), buffImg.getHeight());
				logTime.debug("time base: " + (System.currentTimeMillis() - tmTm)/100);
			}
//			catch (RESyntaxException e)
			catch (Exception e)
			{
				log.error(e);
			}
		}
		return true;
	}


	public void runEngine()
	{
		Pose2D poseLastAcq = null;
		int count = 0;
		int countTStamps = 0;
		long start_tt = System.currentTimeMillis();
		while (count < frames2analyze && poseGrabber.hasNextTimestamp())
		{
			countTStamps++;
			long start_t = System.currentTimeMillis();

			// Itera a posição
			long timestampImg = poseGrabber.getNextTimestamp();
			Pose2D pose = poseGrabber.getPose(timestampImg);
			{
				Pose2D poseMove = poseGrabber.getMovement(timestampImg);
				log.debug("pose = " + pose + ", poseMove = " + poseMove + ", timestampImg = " + timestampImg);
			}

			// Calcula o passo do movimento
			double dLast = 0, dTheta = 0;
			if (poseLastAcq != null)
			{
				dLast = poseLastAcq.getDistance(pose);
				int ang1 = ((int) (poseLastAcq.getTheta()*(180/Math.PI)))%360;
				int ang2 = ((int) (poseLastAcq.getTheta()*(180/Math.PI)))%360;
				dTheta = Math.abs(720 + ang1 - ang2)%360;
			}

			File flImg = new File(flDirImgs, timestampImg + ".bmp");
			if (!flImg.exists())
				flImg = new File(flDirImgs, timestampImg + ".jpeg");
			BufferedImage buffImg = ImageUtil.getImageBMP(flImg);
			int [] inData = ImageUtil.getThreeBandPackedData(buffImg);
			if (outData == null || outData.length < inData.length)
				outData = new int[inData.length];

			log.debug("dLast = " + dLast + ", dTheta = " + dTheta + ", countTStamps = " + countTStamps);
			if (poseLastAcq == null || dLast >= minDistBetweenFrames || dTheta >= minAngleBetweenFrames)
			{
				poseLastAcq = pose;
				imgGrabber.imageReceived(inData, outData, buffImg.getWidth(), buffImg.getHeight(), timestampImg);

				count++;
				poseLast = poseGrabber.getPose(timestampImg);
				poseGrabber.resetMovement();
			}
			log.debug("timeTaken = " + (System.currentTimeMillis() - start_t));
		}
		log.debug("total timeTaken: " + (System.currentTimeMillis() - start_tt));
    }


	public static void main(String [] args)
	{
		try
		{
			if (args.length > 0)
				PlainConfigurator.createConfigurator(new File(args[0]));
			else
				PlainConfigurator.createConfigurator(new File("conf/conf.xml"));

			VLineMatcherRunner estimator = new VLineMatcherRunner();
			if (!estimator.initEngine())
			{
				return;
			}
			estimator.runEngine();
		}
		catch (ConfiguratorException e)
		{
				e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

}

