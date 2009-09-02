package br.com.r4j.research.image.sequence.featurematch.vline;

import java.awt.image.BufferedImage;
import java.util.Arrays;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.morphology.BinaryClosingOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.morphology.BinaryOpeningOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.morphology.LocalMaximaDetector;
import br.com.r4j.image.operation.threebandpacked.morphology.PatternReplacingOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.segmentation.LimiarInputImageOp;
import br.com.r4j.image.operation.threebandpacked.segmentation.RegionCutBySizeOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.DiagonalEliminatorFilterOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.LocalEqualizedLineSobelXDiagonalFilterOneInputImageOp;



/**
 *
 */
public class LineExtractionPreOpImageOp implements ThreeBandPackedOneInputImageOp
{
	private static Log log = LogFactory.getLog(LineExtractionPreOpImageOp.class.getName());
	private static Log logTime = LogFactory.getLog("time");
	
	private LocalEqualizedLineSobelXDiagonalFilterOneInputImageOp sobelLineXOp = null;
	private LimiarInputImageOp limOp = null;
	private LocalMaximaDetector maxDetectorOp = null;

	// Tentativas de eliminar lixo e aumentar as retas ...
	private DiagonalEliminatorFilterOneInputImageOp diagElim = null;
	private PatternReplacingOneInputImageOp patt1Op = null;
	private PatternReplacingOneInputImageOp patt2_1Op = null;
	private PatternReplacingOneInputImageOp patt2_2Op = null;
	private BinaryClosingOneInputImageOp closingOp = null;
	private BinaryOpeningOneInputImageOp openingOp = null;
	private RegionCutBySizeOneInputImageOp smallCutRegs = null;

	private int [] arrayImgPreProc1 = null;
	private int [] arrayImgPreProc2 = null;


	public LineExtractionPreOpImageOp()
	{
		sobelLineXOp = new LocalEqualizedLineSobelXDiagonalFilterOneInputImageOp();
		sobelLineXOp.setModule();

		limOp = new LimiarInputImageOp();
		limOp.setLimiar(200);

		maxDetectorOp = new LocalMaximaDetector();
		maxDetectorOp.setMinValue(50);
		maxDetectorOp.setWidth(5);
		maxDetectorOp.setHeight(1);

		diagElim = new DiagonalEliminatorFilterOneInputImageOp();
		diagElim.setDiagMaxSize(2);

		patt1Op = new PatternReplacingOneInputImageOp();
		patt1Op.setPatterns(
						new int [] {0, 1, 0, 0,
									1, 0, 1, 0,
									0, 1, 0, 0},
						new int [] {0, 0xFFFFFF, 0, 0,
									0, 0xFFFFFF, 0, 0,
									0, 0xFFFFFF, 0, 0},
								4, 3);

		patt2_1Op = new PatternReplacingOneInputImageOp();
		patt2_1Op.setPatterns(
						new int [] {0, 0, 1, 0,
									0, 1, 0, 0,
									0, 0, 1, 0},
						new int [] {0, 0, 0xFFFFFF, 0,
									0, 0, 0xFFFFFF, 0,
									0, 0, 0xFFFFFF, 0,
									0, 0, 0xFFFFFF, 0},
								4, 3);
		patt2_2Op = new PatternReplacingOneInputImageOp();
		patt2_2Op.setPatterns(
						new int [] {0, 1, 0, 0,
									0, 0, 1, 0,
									0, 1, 0, 0},
						new int [] {0, 0xFFFFFF, 0, 0,
									0, 0xFFFFFF, 0, 0,
									0, 0xFFFFFF, 0, 0,
									0, 0xFFFFFF, 0, 0},
								4, 3);
/*
						new int [] {0, 1, 0,
									1, 0, 0,
									0, 1, 0,
									1, 0, 0},
						new int [] {0, 0xFFFFFF, 0,
									0, 0xFFFFFF, 0,
									0, 0xFFFFFF, 0,
									0, 0xFFFFFF, 0},
								3, 4);
//*/

		closingOp = new BinaryClosingOneInputImageOp();
		closingOp.setFilter(new int [] {1,
										1,
										1,
										1,
										1,
										1,
										1,
										1,
										1},
								1, 9);

		openingOp = new BinaryOpeningOneInputImageOp();
		openingOp.setFilter(new int [] {1,
										1,
										1,
										1},
								1, 4);

		smallCutRegs = new RegionCutBySizeOneInputImageOp();
		smallCutRegs.setSizeCut(10);
		smallCutRegs.setB4(true);
	}


	public int operate(int [] inData, int [] outData, int imgWidth, int imgHeight)
	{
		long timeTaken = System.currentTimeMillis();
		if (arrayImgPreProc1 == null || arrayImgPreProc1.length < inData.length)
		{
			arrayImgPreProc1 = new int[inData.length];
			arrayImgPreProc2 = new int[inData.length];
		}
		else
		{
			Arrays.fill(arrayImgPreProc1, 0);
			Arrays.fill(arrayImgPreProc2, 0);
		}
		log.debug("doImagePreparation(1): timeTaken = " + (System.currentTimeMillis() - timeTaken));

		timeTaken = System.currentTimeMillis();
		sobelLineXOp.operate(inData, arrayImgPreProc1, imgWidth, imgHeight);
		logTime.debug("sobelLineXOp: timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc1, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_sobel__" + itCount + ".jpg");
		}

		timeTaken = System.currentTimeMillis();
		maxDetectorOp.operate(arrayImgPreProc1, arrayImgPreProc2, imgWidth, imgHeight);
		logTime.debug("maxDetectorOp: timeTaken = " + (System.currentTimeMillis() - timeTaken));
		log.debug("doImagePreparation(3): timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc2, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_maxdetect__" + itCount + ".jpg");
		}

		timeTaken = System.currentTimeMillis();
		limOp.operate(arrayImgPreProc2, arrayImgPreProc1, imgWidth, imgHeight);
		logTime.debug("limOp: timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc1, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_1_limiar__" + itCount + ".jpg");
		}

		timeTaken = System.currentTimeMillis();
		diagElim.operate(arrayImgPreProc1, outData, imgWidth, imgHeight);
		logTime.debug("diagElim: timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(outData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_diag_elim__" + itCount + ".jpg");
		}
/*
		patt1Op.operate(arrayImgPreProc1, arrayImgPreProc2, imgWidth, imgHeight);

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc2, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_1_2__" + itCount + ".jpg");
		}
		patt2_1Op.operate(arrayImgPreProc2, arrayImgPreProc1, imgWidth, imgHeight);

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc1, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_1_3__" + itCount + ".jpg");
		}
		patt2_2Op.operate(arrayImgPreProc1, arrayImgPreProc2, imgWidth, imgHeight);

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc2, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_1_3_2__" + itCount + ".jpg");
		}
//*/
/*
		smallCutRegs.operate(arrayImgPreProc2, arrayImgPreProc1, imgWidth, imgHeight);
		log.debug("smallCutRegs: timeTaken = " + (System.currentTimeMillis() - timeTaken));
		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc1, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_smallCutRegs__" + itCount + ".jpg");
		}
//		openingOp.operate(arrayImgPreProc2, arrayImgPreProc1, imgWidth, imgHeight);
		closingOp.operate(arrayImgPreProc1, outData, imgWidth, imgHeight);
		log.debug("closingOp: timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(outData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_closing_super__" + itCount + ".jpg");
		}
//*/
/*
		cutRegs.operate(arrayImgPreProc1, arrayImgPreProc2, imgWidth, imgHeight);
		log.debug("doImagePreparation(6): timeTaken = " + (System.currentTimeMillis() - timeTaken));

		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImgPreProc2, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "pre_processed_3__" + itCount + ".jpg");
		}
//*/
		return 0;
	}


	private int itCount = 0;
	public void setItCount(int itCount) {this.itCount = itCount;}
}


