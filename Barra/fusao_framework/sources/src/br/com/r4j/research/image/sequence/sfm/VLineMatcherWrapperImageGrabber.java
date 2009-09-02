package br.com.r4j.research.image.sequence.sfm;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.image.sequence.ImageGrabber;
import br.com.r4j.research.image.sequence.featurematch.vline.Matcher;
import br.com.r4j.research.pose2destimation.Pose2DGrabber;
import br.com.r4j.research.vline.VLineMap;
import br.com.r4j.robosim.Pose2D;


/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 * Também é a classe que coleta os dados. Para os dados assíncronos,
 * ela é chamada por alguém. Para os dados disponíveis sempre, ela
 * chama sincronamente.
 */
public class VLineMatcherWrapperImageGrabber implements ImageGrabber
{
	private static Log log = LogFactory.getLog(VLineMatcherWrapperImageGrabber.class.getName());

	private Matcher matchTest = null;
	private Pose2DGrabber poseGrabber = null;

	private long timeStampLastMeasures = 0;
	

	private int [] arraCopyOut = null;


	public VLineMatcherWrapperImageGrabber()
	{
	}

	
	public void setMatcher(Matcher matcher)
	{
		this.matchTest = matcher;
	}


	public void setPose2DGrabber(Pose2DGrabber poseGrabber)
	{
		this.poseGrabber = poseGrabber;
	}


	public long getMeasuresTimestamp()
	{
		return timeStampLastMeasures;
	}


	public VLineMap getStateMap()
	{
		return matchTest.getStateMap();
	}


	public void imageReceived(int [] inData, int [] outData, int width, int height, long imageTimestamp)
	{
		if (arraCopyOut == null || arraCopyOut.length < inData.length)
		{
			arraCopyOut = new int[inData.length];
		}
		long start_t = System.currentTimeMillis();

		RigidBodyTranformation2D cameraTrafo = new RigidBodyTranformation2D(poseGrabber.getPose(imageTimestamp));
		Pose2D cameraMoveEstimate = poseGrabber.getMovement(imageTimestamp);
		AbstractDoubleSquareMatrix cameraMoveCovar = poseGrabber.getMovementCovar(imageTimestamp);
		AbstractDoubleSquareMatrix cameraPoseCovar = poseGrabber.getPoseCovar(imageTimestamp);
		log.debug("robot pose: " + poseGrabber.getPose(imageTimestamp));
		log.debug("robot move: " + poseGrabber.getMovement(imageTimestamp));



		this.timeStampLastMeasures = imageTimestamp;

		// Calcula as estimativas das linhas conhecidas para o movimento atual.
		// Usa o erro total até agora para tentar repuperar linhas já processadas.
//		matchTest.getMeasures().calculateEstimatedProjections(structurePoseEstimate);

		// Calcula as novas medidas e extrai possíveis novas linhas.
		matchTest.update(inData, width, height, cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar);//, outData);
		log.debug("timeTaken: " + (System.currentTimeMillis() - start_t));
/*
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(outData, width, height, BufferedImage.TYPE_INT_RGB), "out_" + imageTimestamp + ".jpg");
			log.debug("bSaved out_" + imageTimestamp + ".jpg = " + bSaved);
		}
		log.debug("timeTaken after img saving ...: " + (System.currentTimeMillis() - start_t));
//*/
	}
}
