package br.com.r4j.research.image.sequence;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;

import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JRootPane;
import javax.swing.KeyStroke;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PlainConfigurator;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.gui.InputDialogGenerator;
import br.com.r4j.jmr.test.ParaFrenteRespostaSonar;



public class CameraAndPoseAcquisitorRunner
{
	private static Log log = LogFactory.getLog(CameraAndPoseAcquisitorRunner.class.getName());


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

		PropertiesHolder propHldr = Configurator.getPropsHolder();


		final ParaFrenteRespostaSonar poseGrbr = new ParaFrenteRespostaSonar();
		poseGrbr.initEngine();

		CameraAcquisitor camAcq = new CameraAcquisitor();
		camAcq.setDir2Save(propHldr.getStringProperty("img-acq/dir_img"));
		camAcq.setVideoFile(propHldr.getStringProperty("img-acq/file_vid"));
		camAcq.setPoseGrabber(poseGrbr);


		if (!camAcq.initEngine())
		{
			return;
		}
	
		poseGrbr.runEngine();
		camAcq.runEngine();

		InputDialogGenerator diagGen = new InputDialogGenerator(null);
		JDialog diag = diagGen.getDialog();
		JRootPane rootPane = diag.getRootPane();
		ActionListener actionListenerLeft = new ActionListener() { public void actionPerformed(ActionEvent actionEvent) {poseGrbr.goRight();}};
		ActionListener actionListenerRight = new ActionListener() { public void actionPerformed(ActionEvent actionEvent) {poseGrbr.goLeft();}};
		ActionListener actionListenerSpeed = new ActionListener() { public void actionPerformed(ActionEvent actionEvent) {poseGrbr.speed();}};
		ActionListener actionListenerStop = new ActionListener() { public void actionPerformed(ActionEvent actionEvent) {poseGrbr.stop();}};

		KeyStroke strokeLeft = KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0);
		KeyStroke strokeRight = KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0);
		KeyStroke strokeSpeed = KeyStroke.getKeyStroke(KeyEvent.VK_UP, 0);
		KeyStroke strokeStop = KeyStroke.getKeyStroke(KeyEvent.VK_DOWN, 0);
		rootPane.registerKeyboardAction(actionListenerLeft, strokeLeft, JComponent.WHEN_IN_FOCUSED_WINDOW);
		rootPane.registerKeyboardAction(actionListenerRight, strokeRight, JComponent.WHEN_IN_FOCUSED_WINDOW);
		rootPane.registerKeyboardAction(actionListenerSpeed, strokeSpeed, JComponent.WHEN_IN_FOCUSED_WINDOW);
		rootPane.registerKeyboardAction(actionListenerStop, strokeStop, JComponent.WHEN_IN_FOCUSED_WINDOW);

		diagGen.setTitle("Clique para parar!");
		boolean bOK = diagGen.getInput();
		if (bOK)
		{
		}
		poseGrbr.stopEngine();
		camAcq.stopEngine();
		

	}
}
