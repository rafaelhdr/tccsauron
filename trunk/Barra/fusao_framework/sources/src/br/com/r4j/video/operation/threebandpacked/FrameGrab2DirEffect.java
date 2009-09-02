package br.com.r4j.video.operation.threebandpacked;


import java.awt.image.BufferedImage;
import java.io.File;

import javax.media.Buffer;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.research.pose2destimation.Pose2DGrabber;


public class FrameGrab2DirEffect extends ThreeBandPackedBaseEffect
{
	private static Log log = LogFactory.getLog(FrameGrab2DirEffect.class.getName());

	private static Log logDataOutput = LogFactory.getLog(FrameGrab2DirEffect.class.getName() + ".data");
	
	private File flDir2Save = null;

	private long initTime = 0;
	private long lastTime = 0;
	private boolean bGrabFrame = false;

	private Pose2DGrabber poseGrabber = null;
	public void setPoseGrabber(Pose2DGrabber poseGrbr)	{this.poseGrabber = poseGrbr;}


	public FrameGrab2DirEffect()
	{
		super();
		initTime = System.currentTimeMillis();
		lastTime = System.currentTimeMillis();
    }


	public void setThumbnailDir(File flDir2Save)
	{
		this.flDir2Save = flDir2Save;
	}


	protected int doProcess(Buffer inBuffer, Buffer outBuffer, int [] inData, int [] outData, int width, int height)
	{
		long timeAct = System.currentTimeMillis();
		long time = timeAct - initTime;
		log.debug("time from past call: " + (timeAct - lastTime));
		poseGrabber.getPose(time);
		log.debug("t1: " + (System.currentTimeMillis() - timeAct));
		BufferedImage imageBuffer = ImageUtil.createBufferedImage(inData, width, height, BufferedImage.TYPE_INT_RGB);
		log.debug("t2: " + (System.currentTimeMillis() - timeAct));
		File fl2save = new File(flDir2Save, time + ".bmp");
		ImageUtil.saveImageBMP(imageBuffer, fl2save);
		log.debug("t3: " + (System.currentTimeMillis() - timeAct));
		System.arraycopy(inData, 0, outData, 0, inData.length);
		log.debug("t4: " + (System.currentTimeMillis() - timeAct));
		lastTime = System.currentTimeMillis();
		log.debug("time during call: " + (lastTime - timeAct));
        return BUFFER_PROCESSED_OK;
    }

	
    // methods for interface PlugIn
    public String getName()
	{
        return "Grab Frame 2 Dir Man";
    }
}
