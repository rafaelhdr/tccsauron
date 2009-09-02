package br.com.r4j.image.operation.threebandpacked.morphology.ui;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.filter.ui.EffectManager;
import br.com.r4j.gui.InputDialogGenerator;
import br.com.r4j.image.operation.threebandpacked.morphology.LocalMaximaDetector;


public class LocalMaximaDetectorEffectManager implements EffectManager
{
	private static Log log = LogFactory.getLog(LocalMaximaDetectorEffectManager.class.getName());

	private LocalMaximaDetector op = null;

	public LocalMaximaDetectorEffectManager()
	{
		op = new LocalMaximaDetector();
		op.setWidth(5);
		op.setHeight(5);
	}


	public void configure(java.awt.Component cmpParent)
	{
		InputDialogGenerator diagGen = new InputDialogGenerator(cmpParent);
		diagGen.setTitle("Configurar Local Maxima Detector");
		diagGen.addIntegerField("width", "width", op.getWidth(), false);
		diagGen.setFieldLength("width", 3);
		diagGen.addIntegerField("height", "height", op.getHeight(), false);
		diagGen.setFieldLength("height", 3);
		diagGen.addIntegerField("limiar", "limiar", op.getMinValue(), false);
		diagGen.setFieldLength("limiar", 3);
		boolean bOK = diagGen.getInput();
		if (bOK)
		{
			op.setWidth(((Integer) diagGen.getValue("width")).intValue());
			op.setHeight(((Integer) diagGen.getValue("height")).intValue());
			op.setMinValue(((Integer) diagGen.getValue("limiar")).intValue());
		}
	}


	public String getName()
	{
		return "Local Maxima Detector";
	}


	public Object getEffect()
	{
		return op;
	}
}
