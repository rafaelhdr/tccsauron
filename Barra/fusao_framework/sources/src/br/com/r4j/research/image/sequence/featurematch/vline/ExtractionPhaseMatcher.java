
package br.com.r4j.research.image.sequence.featurematch.vline;

import java.awt.image.BufferedImage;
import java.util.Arrays;
import java.util.List;
import java.util.TreeSet;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.SobelVerticalLinesFilter;
import br.com.r4j.research.vline.LineSegmentCollection;
import br.com.r4j.research.vline.VLineProj;


/**
 *
 */
public class ExtractionPhaseMatcher
{
	private static Log log = LogFactory.getLog(ExtractionPhaseMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");
	private static Log logTime = LogFactory.getLog("time");

	
//	public static double LINE_READING_ERROR_BASE = 0.5;
//	public static double LINE_READING_ERROR_BASE = 0.72;
//	public static double LINE_READING_ERROR_BASE = 1.5;
	public static double LINE_READING_ERROR_BASE = 2.2;

	private SobelVerticalLinesFilter lineExtrOp = null;


	///// Buffers temporários
	//
	private int [] arrayTmp = null;


	public ExtractionPhaseMatcher()
	{
		lineExtrOp = new SobelVerticalLinesFilter();

	}


	public void setItCount(int itCount)	
	{
		this.itCount = itCount;
		lineExtrOp.setItCount(itCount);
	}
	private int itCount = 0; // de-de-debug!

	
	public LineExtrationResult lineExtraction(int [] arrayImg, int imgWidth, int imgHeight)
	{
		LineExtrationResult result = new LineExtrationResult();

		if (arrayTmp == null || arrayTmp.length < arrayImg.length)
			arrayTmp = new int[arrayImg.length];
		else
			Arrays.fill(arrayTmp, 0);

		//////////
		// extração das retas
		// 
		long timeTaken = System.currentTimeMillis();
		lineExtrOp.operate(arrayImg, arrayTmp, imgWidth, imgHeight);
		if (log.isDebugEnabled())
			log.debug("lineExtraction:doImagePreparation: timeTaken = " + (System.currentTimeMillis() - timeTaken));

		TreeSet setMeasuresTmp = new TreeSet();
		List listLines = lineExtrOp.getLines();
		result.arrayProjMeasures = new VLineProj[listLines.size()];

		if (log.isDebugEnabled())
		{
			if (arrayTmp == null || arrayTmp.length < arrayImg.length)
				arrayTmp = new int[arrayImg.length];
			else
				Arrays.fill(arrayTmp, 0);

			log.debug("result.arrayProjMeasures.length: " + result.arrayProjMeasures.length);
			log.debug("imgWidth: " + imgWidth + ", imgHeight: " + imgHeight);
			log.debug("arrayImg.length: " + arrayImg.length);
			log.debug("arrayTmp.length: " + arrayTmp.length);
		}
		for (int i = 0; i < result.arrayProjMeasures.length; i++)
		{
			LineSegmentCollection lCol = (LineSegmentCollection) listLines.get(i);
			
			double lineError = LINE_READING_ERROR_BASE/Math.sqrt(lCol.sizeSeg/2.0);
//			double lineError = LINE_READING_ERROR_BASE;
/*
			if (lineError < 0.5)
				lineError = 0.5;
/*/
//			if (lineError < 0.9)
				lineError = 0.2;
//*/

			VLineProj vLineP = new VLineProj(lCol, lineError);
			vLineP.calculatePerfil(arrayImg, imgWidth, imgHeight);

			result.arrayProjMeasures[i] = vLineP;

			// Debug ...
			/*PRINTIMGS
			if (log.isDebugEnabled())
			{
				int [] arrayRed = result.arrayProjMeasures[i].getPerfil().getPerfilRed();
				int [] arrayGreen = result.arrayProjMeasures[i].getPerfil().getPerfilGreen();
				int [] arrayBlue = result.arrayProjMeasures[i].getPerfil().getPerfilBlue();
				int uBeginReal = (int) (result.arrayProjMeasures[i].getU() - arrayBlue.length / 2);
				int uBegin = uBeginReal;
				if (uBegin < 0)	uBegin = 0;
				int uEnd = uBegin + arrayBlue.length;
				if (uEnd > imgWidth)	uEnd = imgWidth;
				StringBuffer strBuff = new StringBuffer();

				int mult = imgHeight/result.arrayProjMeasures.length;
				log.debug("mult: " + mult);
				for (int j = uBegin; j < uEnd; j++)
				{
					arrayTmp[((i*mult)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 1)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 2)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 3)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 4)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 5)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 6)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 7)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 8)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					arrayTmp[((i*mult + 9)%(imgHeight-10))*imgWidth + j] = ThreeBandPackedUtil.getPackedPixel(arrayRed[j-uBegin], arrayGreen[j-uBegin], arrayBlue[j-uBegin]);
					strBuff.append("(");
					strBuff.append(arrayRed[j-uBegin]);
					strBuff.append(",");
					strBuff.append(arrayGreen[j-uBegin]);
					strBuff.append(",");
					strBuff.append(arrayBlue[j-uBegin]);
					strBuff.append("),");
				}
//				log.debug("result.arrayProjMeasures[i].getU(): " + result.arrayProjMeasures[i].getU());
				arrayTmp[((i*mult + 10)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 11)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 12)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 13)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 14)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 15)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				arrayTmp[((i*mult + 16)%(imgHeight-10))*imgWidth + (int) result.arrayProjMeasures[i].getU()] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				log.debug(result.arrayProjMeasures[i].getU() + ":" + strBuff.toString());
			}
			//*/
		}

		/*PRINTIMGS
		if (log.isDebugEnabled())
		{
			log.debug("prep medicoes: timeTaken = " + (System.currentTimeMillis() - timeTaken));
			boolean bSaved = ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayTmp, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "perfis_" + itCount + ".jpg");
			log.debug("bSaved perfis_" + itCount + ".jpg = " + bSaved);
			log.debug("after img saving ...: timeTaken = " + (System.currentTimeMillis() - timeTaken));
		}
		//*/
		return result;
	}
}