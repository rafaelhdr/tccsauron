package br.com.r4j.image.operation.threebandpacked.spatialfilter.ui;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.filter.ui.EffectManager;
import br.com.r4j.gui.InputDialogGenerator;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.SobelVerticalLinesFilter;



public class SobelVerticalLinesffectManager implements EffectManager
{
	private static Log log = LogFactory.getLog(SobelVerticalLinesffectManager.class.getName());

	private SobelVerticalLinesFilter op = null;

	public SobelVerticalLinesffectManager()
	{
		op = new SobelVerticalLinesFilter();
	}


	public void configure(java.awt.Component cmpParent)
	{
		InputDialogGenerator diagGen = new InputDialogGenerator(cmpParent);
		diagGen.setTitle("Configurar " + this.getName());

		diagGen.addIntegerField("minSegmentLength", op.getMinSegmentLengthName(), op.getMinSegmentLength(), false);
		diagGen.setFieldLength("minSegmentLength", 6);

		diagGen.addIntegerField("minLineLength", op.getMinLineLengthName(), op.getMinLineLength(), false);
		diagGen.setFieldLength("minLineLength", 6);

		diagGen.addDoubleField("meanPercHigher", op.getMeanPercHigherName(), op.getMeanPercHigher(), false);
		diagGen.setFieldLength("meanPercHigher", 6);

		diagGen.addDoubleField("meanPercLower", op.getMeanPercLowerName(), op.getMeanPercLower(), false);
		diagGen.setFieldLength("meanPercLower", 6);

		diagGen.addDoubleField("minM", op.getMinMName(), op.getMinM(), false);
		diagGen.setFieldLength("minM", 6);

		diagGen.addDoubleField("maxM", op.getMaxMName(), op.getMaxM(), false);
		diagGen.setFieldLength("maxM", 6);

		diagGen.addIntegerField("tresholdChangeDir", op.getTresholdChangeDirName(), op.getTresholdChangeDir(), false);
		diagGen.setFieldLength("tresholdChangeDir", 6);

		diagGen.addIntegerField("maxBlindTries", op.getMaxBlindTriesName(), op.getMaxBlindTries(), false);
		diagGen.setFieldLength("maxBlindTries", 6);

		diagGen.addIntegerField("tresholdZero", op.getTresholdZeroName(), op.getTresholdZero(), false);
		diagGen.setFieldLength("tresholdZero", 6);

		diagGen.addDoubleField("ratioDH_DV", op.getRatioDH_DVName(), op.getRatioDH_DV(), false);
		diagGen.setFieldLength("ratioDH_DV", 6);

		boolean bOK = diagGen.getInput();
		if (bOK)
		{
			op.setMinSegmentLength((int) ((Integer) diagGen.getValue("minSegmentLength")).intValue());
			op.setMinSegmentLength((int) ((Integer) diagGen.getValue("minSegmentLength")).intValue());
			op.setMinLineLength((int) ((Integer) diagGen.getValue("minLineLength")).intValue());
			op.setMeanPercHigher((double) ((Double) diagGen.getValue("meanPercHigher")).doubleValue());
			op.setMeanPercLower((double) ((Double) diagGen.getValue("meanPercLower")).doubleValue());
			op.setMinM((double) ((Double) diagGen.getValue("minM")).doubleValue());
			op.setMaxM((double) ((Double) diagGen.getValue("maxM")).doubleValue());
			op.setTresholdChangeDir((int) ((Integer) diagGen.getValue("tresholdChangeDir")).intValue());
			op.setMaxBlindTries((int) ((Integer) diagGen.getValue("maxBlindTries")).intValue());
			op.setTresholdZero((int) ((Integer) diagGen.getValue("tresholdZero")).intValue());
			op.setRatioDH_DV((double) ((Double) diagGen.getValue("ratioDH_DV")).doubleValue());
		}
	}


	public String getName()
	{
		return "Extrator de Linhas Quase verticais";
	}


	public Object getEffect()
	{
		return op;
	}
}
