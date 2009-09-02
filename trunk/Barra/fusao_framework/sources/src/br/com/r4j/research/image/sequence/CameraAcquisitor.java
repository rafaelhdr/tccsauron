package br.com.r4j.research.image.sequence;

import java.io.File;
import java.io.IOException;

import javax.media.Controller;
import javax.media.ControllerEvent;
import javax.media.ControllerListener;
import javax.media.Processor;
import javax.media.StopByRequestEvent;
import javax.media.protocol.DataSource;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.research.pose2destimation.Pose2DGrabber;
import br.com.r4j.studio.video.CameraDataSourceManager;
import br.com.r4j.studio.video.ComplexDataSourceManager;
import br.com.r4j.studio.video.EffectProcessorManager;
import br.com.r4j.studio.video.FileDataSinkManager;
import br.com.r4j.studio.video.ProcessorManager;
import br.com.r4j.video.operation.threebandpacked.FrameGrab2DirEffect;



public class CameraAcquisitor implements Runnable, ControllerListener
{
	private static Log log = LogFactory.getLog(CameraAcquisitor.class.getName());

	private Pose2DGrabber poseGrabber = null;

	private String strDir2Save = null;
	private File flDir2Save = null;
	private String strVideoFile = null;

	private CameraDataSourceManager camSource = null;
	private ProcessorManager procMan = null;
	private FileDataSinkManager fileSinkMan = null;
	private FrameGrab2DirEffect copy2dirEffect = null;	
	private boolean bIsDSConnected = false;
	private boolean bIsProcessorCreated = false;
	private boolean bIsGraphConnected = false;


	private boolean bRunning = false;



	public CameraAcquisitor()
	{
	}

	
	public void setPoseGrabber(Pose2DGrabber poseGrbr)	{this.poseGrabber = poseGrbr;}
	public void setDir2Save(String strDir2Save)	
	{
		this.strDir2Save = strDir2Save;
		this.flDir2Save = new File(strDir2Save);
		if (!this.flDir2Save.exists())
			this.flDir2Save.mkdirs();
	}

	public void setVideoFile(String strVideoFile)	{this.strVideoFile = strVideoFile;}


	public boolean initEngine()
	{
		try
		{
			camSource = new CameraDataSourceManager();
			camSource.open();

			procMan = new EffectProcessorManager();

			copy2dirEffect = new FrameGrab2DirEffect();
			copy2dirEffect.setThumbnailDir(flDir2Save);
			copy2dirEffect.setPoseGrabber(poseGrabber);

			fileSinkMan = new FileDataSinkManager();
			log.debug("strVideoFile = " + strVideoFile);
			fileSinkMan.setFileName(strVideoFile);

			return true;
		}
		catch (IOException e)
		{
			e.printStackTrace();
			log.debug(e);

			return false;
		}
	}


	public void runEngine()
	{
		bRunning = true;
		Thread thr = new Thread(this);
		thr.start();
	}

	
	public void run()
	{
		log.debug("run:begin");

		Processor proc = null;
		try
		{
			camSource.open();
			bIsDSConnected = true;
			camSource.start();
			DataSource dSourceOri = camSource.getDataSource();

			procMan.setContentDescriptor(null);
			procMan.setFilters(br.com.r4j.commons.util.Collections.createList(copy2dirEffect));
			procMan.setDataSource(dSourceOri);
			proc = procMan.getProcessor();

			if (proc == null)
			{
				log.error("Não foi possível iniciar o processador: erro interno");
				this.resetAll();
				return;
			}

			proc.addControllerListener(this);
			procMan.open();
			procMan.start();
			if (camSource instanceof ComplexDataSourceManager)
				((ComplexDataSourceManager) camSource).afterStartPorcessorStart();
			bIsProcessorCreated = true;
		}
		catch (Exception e)
		{
			log.error("startGraph:error", e);
			this.resetAll();
			return;
		}

		try
		{
			fileSinkMan.setDataSource(camSource.getClone());
			fileSinkMan.open();
			fileSinkMan.start();
		}
		catch (Exception e)
		{
			log.error("startGraph:error", e);
				this.resetAll();
				return;
		}
		bIsGraphConnected = true;
	}


	public void resetAll()
	{
		if (bIsProcessorCreated)
		{
			log.debug("stopGraph:begin");
			try
			{
				fileSinkMan.stop();
				fileSinkMan.close();
			}
			catch (Exception e)
			{
				e.printStackTrace();
				log.error("stopGraph:error", e);
			}
			try
			{
				if (procMan != null)
				{
					procMan.stop();
					procMan.getProcessor().removeControllerListener(this);
					procMan.close();
				}
				camSource.stop();
			}
			catch (Exception e)
			{
				e.printStackTrace();
				log.error("stopGraph:error", e);
			}
		}
		bIsGraphConnected = false;
		bIsProcessorCreated = false;
		log.debug("stopGraph:end");
	}


	public void stopEngine()
	{
		bRunning = false;
		this.resetAll();
	}


	///////////////////////////////////
	//////// ControllerListener ///////
	///////////////////////////////////

    public synchronized void controllerUpdate(ControllerEvent event)
	{
		log.debug("controllerUpdate:event = " + event);
        Processor proc = null;
        Controller controller = (Controller) event.getSource();
		System.err.println("controllerUpdate:controller = " + controller);
		log.debug("controllerUpdate:controller = " + controller);
		if (controller != null)
			log.debug("controllerUpdate:controller.getClass().getName() = " + controller.getClass().getName());
        if (controller instanceof Processor)
            proc = (Processor) controller;

        if (event instanceof StopByRequestEvent)
		{
			log.debug("controllerUpdate:event instanceof StopByRequestEvent");
			// Não pode pois pause cai aqui ...
////			ui.setVisualAndControlComponent(null, null);
        }		
    }
}
