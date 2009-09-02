package br.com.r4j.image.operation.threebandpacked.spatialfilter;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.TreeSet;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.image.operation.threebandpacked.colorspace.BitMaskLookupFilter;
import br.com.r4j.image.operation.threebandpacked.colorspace.HistogramEqualizationOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.colorspace.LookupFilter;
import br.com.r4j.research.vline.LineSegHolder;
import br.com.r4j.research.vline.LineSegment;
import br.com.r4j.research.vline.LineSegmentCollection;


/** 
 * Passos de execução:
 *
 *
 */
public class SobelVerticalLinesFilter implements ThreeBandPackedOneInputImageOp
{
	protected static Log log = LogFactory.getLog(SobelVerticalLinesFilter.class.getName());

	private static int LINE_CENTER = 1;
	private static int VISITED = 2;
	private static int REJECTED_OUT_OF_SPEC = 4;
	private static int REJECTED_TOO_CLOSE = 8;
	private static int VISITED_UNKNOWN = 16;
	private static int VISITED_UNKNOWN_2 = 32;
	private static int LINE_PATH = 64 | LINE_CENTER;


	public static int X_FIRST = 0;
	public static int Y_FIRST = 1;
	public static int X_LAST = 2;
	public static int Y_LAST = 3;
	public static int VAL = 4;
	public static int LENGTH = 5;
	public static int JUMP = 6;


	private int [] tmpData = null;
	private int [] tmp2Data = null;
	private int [] tmpOutData = null;
	private int [] tmpOut2Data = null;
	private int [] tmpOut3Data = null;
	private int [] maxOutData = null;
	private int [] arrayOutDebug = null;
	private int [] weightData = null;

	private SpatialFilterOneInputImageOp spatialOp = null;
	private int [] meanData = null;
	private float [] meanFilter = null;

	
	/**
	 * Cada pixel representa um pixel da imagem, mesma posição.
	 *
	 *  LINE_CENTER (1):  inicio de um possível segmento
	 *  VISITED (2):  já visitado.
	 *  REJECTED_OUT_OF_SPEC (4):  centro rejeitado por estar fora de especificação.
	 *  REJECTED_TOO_CLOSE (8):  centro rejeitado por algum ponto pesquisado cair muito próximo a outra linha.
	 *  UNKNOWN (16): 
	 *
	 */
	private int [] maskData = null;

	/**
	 * Matriz do tamanho da de imagem, mas tansposta.
	 *
	 * Para cada linha, as duas primeiras posições tem (i, j) indicando o ponto inicial
	 * na imagem que a linha começa.
	 *
	 * Seguindo na linha, valores -1, 0 ou 1 indicando a inclinação do pixel em relação ao
	 * anterior. Cada ponto da linah indica um ponto com a mesma altura indo para baixo na
	 * imagem a partir do ponto inicial. A sequencia acaba com um valor 0xFF.
	 *
	 * No fim da linha, o mesmo padrão existe, só que indo do fim para o começo, indicando
	 * os pontos acima do ponto central.
	 *
	 */
	private int [] inclTmpData = null;
	private int [] inclData = null;
	private int [] inclSoftData = null;


	/**
	 * Armazena valores de retorno.
	 */
	private ArrayList listLineParams = null;
	public  ArrayList getLines()	{return listLineParams;}

	// Gera ou não as magens de debug intermediárias.
	private boolean bGenerateOutput = false;
	public void setGenerateOutput(boolean a)	{this.bGenerateOutput = a;}
	public  boolean getGenerateOutput()	{return bGenerateOutput;}

	// Tamanho mínimo de um segmento considerado.
	private int minSegmentLength = 0;
	public void setMinSegmentLength(int a)	{this.minSegmentLength = a;}
	public int getMinSegmentLength()	{return minSegmentLength;}
	public String getMinSegmentLengthName()	{return "Segmento mínimo";}

	// Tamanho mínimo de uma linha
	private int minLineLength = 0;
	public void setMinLineLength(int a)	{this.minLineLength = a;}
	public int getMinLineLength()	{return minLineLength;}
	public String getMinLineLengthName()	{return "Linha mínima";}

	// Valor mínimo para começar a definir uma linha
	private int tresholdHigher = 0;
	private double meanPercHigher = 0;
	public void setMeanPercHigher(double a)	{this.meanPercHigher = a;}
	public double getMeanPercHigher()	{return meanPercHigher;}
	public String getMeanPercHigherName()	{return "Valor mínimo de início";}


	// Valor mínimo para parar de definir uma linha
	private int tresholdLower = 0;
	private double meanPercLower = 0;
	public void setMeanPercLower(double a)	{this.meanPercLower = a;}
	public double getMeanPercLower()	{return meanPercLower;}
	public String getMeanPercLowerName()	{return "Valor mínimo de parada";}

	private double minM = 0;
	public void setMinM(double a)	{this.minM = a;}
	public double getMinM()	{return minM;}
	public String getMinMName()	{return "Incinação mínima";}

	private double maxM = 0;
	public void setMaxM(double a)	{this.maxM = a;}
	public double getMaxM()	{return maxM;}
	public String getMaxMName()	{return "Inclinação máxima";}

	// Usado para dar preferência a seguir na direção vertical ao extrair as 
	// linhas das imagens de diferença. Quanto maior o valor, maior a tendencia
	// a seguir a direção vertical mesmo tendo direções com intensidades mais
	// fortes.
	private int tresholdChangeDir = 0;
	public void setTresholdChangeDir(int a)	{this.tresholdChangeDir = a;}
	public int getTresholdChangeDir()	{return tresholdChangeDir;}
	public String getTresholdChangeDirName()	{return "Inercia Vertical";}

	// Indica o número de pixels que itera para gerar as linhas a partir dos 
	// máximos do resultado das operações de diferença sem encontrar o valor
	// mínimo imposto por tresholdLower.
	private int maxBlindTries = 0;
	public void setMaxBlindTries(int a)	{this.maxBlindTries = a;}
	public int getMaxBlindTries()	{return maxBlindTries;}
	public String getMaxBlindTriesName()	{return "N. de iterações as cegas";}

	// Indica o valor mínimo considerado. Abaixo disso, o valor é zerado após as operações de diferenças.
	private int tresholdZero = 0;
	public void setTresholdZero(int a)	{this.tresholdZero = a;}
	public int getTresholdZero()	{return tresholdZero;}
	public String getTresholdZeroName()	{return "Nivel mínimo de cinza";}

	// A relação mínima entre a derivada vertical e horizontal de um ponto para que esse 
	// possa ser considerado.
	private double ratioDH_DV = 0;
	public void setRatioDH_DV(double a)	{this.ratioDH_DV = a;}
	public double getRatioDH_DV()	{return ratioDH_DV;}
	public String getRatioDH_DVName()	{return "Relação mínima DH/DV";}


	private int itCount = 0;
	public void setItCount(int itCount) {this.itCount = itCount;}

/*PRINTIMGS
	private HistogramEqualizationOneInputImageOp histOp = null;
	private BitMaskLookupFilter bitMaskLookupFilter = null;
	private LookupFilter lookupFilter = null;
	private int [] arraColorMaskIn = null;
	private int [] arraColorMaskOut = null;
	private int [] arraColorInclIn = null;
	private int [] arraColorInclOut = null;
	private int [] arraColorInclSoftIn = null;
	private int [] arraColorInclSoftOut = null;
//*/


	public SobelVerticalLinesFilter()
	{
		listLineParams = new ArrayList();

		bGenerateOutput = true;

		meanPercHigher = 0.20;
		meanPercLower = 0.48;

		tresholdChangeDir = 1;
		maxBlindTries = 6;

		tresholdZero = 9;//4;

		ratioDH_DV = 5.0;

		minSegmentLength = 15;
		minLineLength = 30;

		double thetaRange = 7.5;
		maxM = Math.atan((Math.PI * (thetaRange/2.0) / 180));
//		minM = -maxM;

		spatialOp = new SpatialFilterOneInputImageOp();
		float size = 3*5;
		float [] meanFilterTmp = {(float) 1.0/size, (float) 1.0/size, (float) 1.0/size,
								  (float) 1.0/size, (float) 1.0/size, (float) 1.0/size,
								  (float) 1.0/size, (float) 1.0/size, (float) 1.0/size,
								  (float) 1.0/size, (float) 1.0/size, (float) 1.0/size,
								  (float) 1.0/size, (float) 1.0/size, (float) 1.0/size
		};
		meanFilter = meanFilterTmp;
	}


	public int operate(int [] inData, int [] outData, int imgWidth, int imgHeight)
	{
		/*PRINTIMGS
		log.debug("-- "+itCount+" -----------------------------------------------------------------");
		ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(inData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_original.bmp");
		//*/

		//////////
		// Inicia 
		int max = imgWidth*imgHeight;
		if (tmpData == null || tmpData.length < inData.length)
		{
			tmpData = new int[inData.length];
			tmp2Data = new int[inData.length];
			maskData = new int[inData.length];
			inclData = new int[inData.length];
			inclTmpData = new int[inData.length];
			inclSoftData = new int[inData.length];
			meanData = new int[inData.length];

			
			/*PRINTIMGS
			maxOutData = new int[inData.length];
			tmpOutData = new int[inData.length];
			tmpOut2Data = new int[inData.length];
			tmpOut3Data = new int[inData.length];
			arrayOutDebug = new int[inData.length];
			//*/

			weightData = new int[inData.length];
		}
	
		/*PRINTIMGS
		if (bGenerateOutput)
		{
			if (histOp == null)
			{
				histOp = new HistogramEqualizationOneInputImageOp();

				lookupFilter = new LookupFilter();
				bitMaskLookupFilter = new BitMaskLookupFilter();

				int [] arraColorMaskInTmp = {LINE_CENTER, VISITED, REJECTED_OUT_OF_SPEC,
											 REJECTED_TOO_CLOSE, VISITED_UNKNOWN, VISITED_UNKNOWN_2, 
											 LINE_PATH};
				arraColorMaskIn = arraColorMaskInTmp;

				arraColorMaskOut = new int[7];
				arraColorMaskOut[0] = 0xFFFFFF;
				arraColorMaskOut[1] = 0xFF00FF;
				arraColorMaskOut[2] = 0x0000FF;
				arraColorMaskOut[3] = 0xFF0000;
				arraColorMaskOut[4] = 0x00FFFF;
				arraColorMaskOut[5] = 0xFFFF00;
				arraColorMaskOut[6] = 0x9F9F9F;

				arraColorInclIn = new int[5];
				arraColorInclIn[0] = -1;
				arraColorInclIn[1] = 0;
				arraColorInclIn[2] = 1;
				arraColorInclIn[3] = 0xFF;
				arraColorInclIn[4] = 5;

				arraColorInclOut = new int[5];
				arraColorInclOut[0] = 0xFFFFFF;
				arraColorInclOut[1] = 0x07F7F7F;
				arraColorInclOut[2] = 0x000000;
				arraColorInclOut[3] = 0xFF00FF;
				arraColorInclOut[4] = 0xFF0000;

				int [] arraColorInclSoftInTmp = {-7, -6, -5, -4,
												 -3, -2, -1,  0,
												  1,  2,  3,  4,
												  5,  6,  7,  0xFF};
				arraColorInclSoftIn = arraColorInclSoftInTmp;

				int [] arraColorInclSoftOutTmp = {0x00FF00, 0x00FF00, 0x00FF00, 0x4F4F00, 
												  0x9FFF00, 0xFFFF00, 0x6F6F6F, 0xAFAFAF, 
												  0xFFFFFF, 0x00FFFF, 0x009FFF, 0x004F4F, 
												  0xFF0000, 0xFF0000, 0xFF0000, 0x000000};
				arraColorInclSoftOut = arraColorInclSoftOutTmp;
			}
		}
		if (log.isDebugEnabled())
		{
			boolean bSaved = ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(inData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_0.bmp");
		}
		//*/

		//////////
		// Inicia matrizes 
		{
			int red = 0, green = 0, blue = 0;
			int maxWidth = imgWidth, maxHeight = imgHeight;
			for (int j = 0; j < maxHeight; j++)
			{
				int lineStride = imgWidth*j;
				for (int i = 0; i < maxWidth; i++)
				{
					int idx = lineStride + i;

					// zera array auxiliares
					maskData[idx] = 0; tmp2Data[idx] = 0; 
					inclTmpData[idx] = 0xFFFF; inclData[idx] = 0xFFFF; inclSoftData[idx] = 0xFFFF;
					
					/*PRINTIMGS
					maxOutData[idx] = 0;
					//*/
				}
			}
		}


		//////////
		// Suaviza. Todo mundo faz isso, deve funcionar ...
		{
			spatialOp.setFilter(meanFilter, 3, 5);
			spatialOp.operate(inData, meanData, imgWidth, imgHeight);
			/*PRINTIMGS
			if (log.isDebugEnabled())
			{
				boolean bSaved = ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(meanData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_00.bmp");
			}
			//*/
		}


		//////////
		// Calcula imagem das diferenças e os tresholds.
		{
			int mean = 0, maxVal = 0, meanCount = 0;
			int redHoriz, redVert, greenHoriz, blueHoriz, greenVert, blueVert;
			int maxWidth = imgWidth - 2, maxHeight = imgHeight - 2;
			for (int j = 2; j < maxHeight; j++)
			{
				int lineStride = imgWidth*j;

				for (int i = 2; i < maxWidth; i++)
				{
					int idx = lineStride + i;

					int idxLeft1 = idx - 1, idxRight1 = idx + 1;
					int idxLeft2 = idx - 2, idxRight2 = idx + 2;
					redHoriz = ((meanData[idxRight1]&REDMASK)>>REDSHIFT) - ((meanData[idxLeft1]&REDMASK)>>REDSHIFT); 
					redHoriz += ((meanData[idxRight2]&REDMASK)>>REDSHIFT) - ((meanData[idxLeft2]&REDMASK)>>REDSHIFT); 
					if (redHoriz < 0) redHoriz = -redHoriz;
					redHoriz /= 2;

					greenHoriz = ((meanData[idxRight1]&GREENMASK)>>GREENSHIFT) - ((meanData[idxLeft1]&GREENMASK)>>GREENSHIFT); 
					greenHoriz += ((meanData[idxRight2]&GREENMASK)>>GREENSHIFT) - ((meanData[idxLeft2]&GREENMASK)>>GREENSHIFT); 
					if (greenHoriz < 0) greenHoriz = -greenHoriz;
					greenHoriz /= 2;

					blueHoriz = ((meanData[idxRight1]&BLUEMASK)>>BLUESHIFT) - ((meanData[idxLeft1]&BLUEMASK)>>BLUESHIFT); 
					blueHoriz += ((meanData[idxRight2]&BLUEMASK)>>BLUESHIFT) - ((meanData[idxLeft2]&BLUEMASK)>>BLUESHIFT); 
					if (blueHoriz < 0) blueHoriz = -blueHoriz;
					blueHoriz /= 2;

					int idxTop1 = idx - imgWidth, idxBottom1 = idx + imgWidth;
					int idxTop2 = idx - imgWidth*2, idxBottom2 = idx + imgWidth*2;

					redVert = ((meanData[idxTop1]&REDMASK)>>REDSHIFT) - ((meanData[idxBottom1]&REDMASK)>>REDSHIFT); 
					redVert += ((meanData[idxTop2]&REDMASK)>>REDSHIFT) - ((meanData[idxBottom2]&REDMASK)>>REDSHIFT); 
					if (redVert < 0) redVert = -redVert;
					redVert /= 2;
					
					greenVert = ((meanData[idxTop1]&GREENMASK)>>GREENSHIFT) - ((meanData[idxBottom1]&GREENMASK)>>GREENSHIFT); 
					greenVert += ((meanData[idxTop2]&GREENMASK)>>GREENSHIFT) - ((meanData[idxBottom2]&GREENMASK)>>GREENSHIFT); 
					if (greenVert < 0) greenVert = -greenVert;
					greenVert /= 2;

					blueVert = ((meanData[idxTop1]&BLUEMASK)>>BLUESHIFT) - ((meanData[idxBottom1]&BLUEMASK)>>BLUESHIFT); 
					blueVert += ((meanData[idxTop2]&BLUEMASK)>>BLUESHIFT) - ((meanData[idxBottom2]&BLUEMASK)>>BLUESHIFT); 
					if (blueVert < 0) blueVert = -blueVert;
					blueVert /= 2;

/*					
					if (redHoriz < greenHoriz) {redHoriz = greenHoriz; redVert = greenVert;}
					if (redHoriz < blueHoriz) {redHoriz = blueHoriz; redVert = blueVert;}
/*/
					redHoriz += greenHoriz + blueHoriz;
					redVert += greenVert + blueVert;
//*/
					weightData[idx] = redHoriz;

					/*PRINTIMGS
					if (log.isDebugEnabled()) if (redVert != 0)
					{
						tmpOut2Data[idx] = redHoriz;
						tmpOut3Data[idx] = redVert;
						tmpOutData[idx] = 5*redHoriz/redVert;
						if (tmpOutData[idx] > 255)
							tmpOutData[idx] = 0xFFFFFF;
					}
					else
						tmpOutData[idx] = redHoriz << 8;
					//*/

					if (redHoriz > tresholdZero && (redVert == 0 || redHoriz > ratioDH_DV*redVert))
					{
						/*PRINTIMGS
						if (redHoriz > maxVal)
							maxVal = redHoriz;
						//*/

						meanCount++;

						// diferença horizontal multiplicada pela relação diff horiz / diff vert
						int tmpVal = redHoriz - redVert;
//						int tmpVal = redHoriz * redHoriz / (redVert + 1) / 5;
//						if (tmpVal > 255) 
//							tmpVal = 255;
						mean += tmpVal;
						tmpData[idx] = tmpVal;
					}
					else
						tmpData[idx] = 0;
				}
			}
			if (meanCount > 0)
				mean /= meanCount;
			tresholdLower = (int) (mean*1.0*(1 - meanPercLower));
			tresholdHigher = (int) (mean*1.0*(1 + meanPercHigher));

			/*PRINTIMGS
			if (log.isDebugEnabled())
			{
				log.debug("meanPercLower = " + meanPercLower + ", meanPercHigher = " + meanPercHigher);
				log.debug("mean = " + mean + ", tresholdLower = " + tresholdLower + ", tresholdHigher = " + tresholdHigher + ", maxVal = " + maxVal);
			}
			//*/
		}

		/*PRINTIMGS
		if (bGenerateOutput)
		{
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOut2Data, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_1_1_2_horiz.bmp");
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOut3Data, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_1_1_2_vert.bmp");
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_1_1_2_rel_vh.bmp");
			histOp.operate(tmpData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_1_1.bmp");
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_1.bmp");
		}
//*/

		//////////
		// Acaba de calcular a imagem de diferenças na direção vertical.
		int wndHeight = 3;
		int valMax = 1;
		{
			int maxHeight = imgHeight - wndHeight, maxWidth = imgWidth;
			for (int j = wndHeight; j < maxHeight; j++)
			{
				int lineStride = imgWidth*j;
				for (int i = 0; i < maxWidth; i++)
				{
					int idx = lineStride + i;
					int idxUp1 = idx - imgWidth, idxDown1 = idx + imgWidth;
					int idxUp2 = idxUp1 - imgWidth, idxDown2 = idxDown1 + imgWidth;
					int idxUp3 = idxUp2 - imgWidth, idxDown3 = idxDown2 + imgWidth;
//					int idxUp4 = idxUp3 - imgWidth, idxDown4 = idxDown3 + imgWidth;
/*
					int smallerUp = tmpData[idxUp1];
					if (smallerUp > tmpData[idxUp2]) smallerUp = tmpData[idxUp2];
					if (smallerUp > tmpData[idxUp3]) smallerUp = tmpData[idxUp3];
					if (smallerUp > tmpData[idxUp4]) smallerUp = tmpData[idxUp4];
					int smallerDown = tmpData[idxDown1];
					if (smallerDown > tmpData[idxDown2]) smallerDown = tmpData[idxDown2];
					if (smallerDown > tmpData[idxDown3]) smallerDown = tmpData[idxDown3];
					if (smallerDown > tmpData[idxDown4]) smallerDown = tmpData[idxDown4];

					tmp2Data[idx] = (tmpData[idx] + smallerDown + smallerUp) / 3;
/*/
//					int valUp = (tmpData[idxUp1] + tmpData[idxUp2] + tmpData[idxUp3] + tmpData[idxUp4])/wndHeight;
//					int valDown = (tmpData[idxDown1] + tmpData[idxDown2] + tmpData[idxDown3] + tmpData[idxDown4])/wndHeight;
					int valUp = (tmpData[idxUp1] + tmpData[idxUp2] + tmpData[idxUp3])/wndHeight;
					int valDown = (tmpData[idxDown1] + tmpData[idxDown2] + tmpData[idxDown3])/wndHeight;
//					int valUp = (tmpData[idxUp1] + tmpData[idxUp2])/wndHeight;
//					int valDown = (tmpData[idxDown1] + tmpData[idxDown2])/wndHeight;

					if (valUp < tresholdLower && valDown < tresholdLower)
						tmp2Data[idx] = 0;
					else
					{
//*
						int valBig = valUp;
						if (valBig < valDown)
							valBig = valDown;
						if (valBig < tmpData[idx])
							valBig = tmpData[idx];
//						tmp2Data[idx] = valBig;

//*/
//						tmp2Data[idx] = (valBig + valUp + valDown + tmpData[idx]) / 4;
						tmp2Data[idx] = (valUp*wndHeight + valDown*wndHeight + tmpData[idx]) / (2*wndHeight + 1);
						if (tmp2Data[idx] > valMax)
							valMax = tmp2Data[idx];
					}
//*/
				}
			}
		}

/*
6000
1000
1700
15000
-----
24000

 http://wcam37/niku/app?action=ecb_captura_projetos_pop&
 unit_id=5000147&
 pn=1&
 grupobraskemmais=1&
 braskemmais=1&
 mes&
 status_validacao&
 status_captura&tipoempreendimen&status_metas&status_combo&ano
 //*/

		/*PRINTIMGS
		if (bGenerateOutput)
		{
			histOp.operate(tmp2Data, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmp2Data, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_2_1.bmp");
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_2.bmp");
		}
		{
			int maxHeight = imgHeight - wndHeight, maxWidth = imgWidth;
			for (int j = wndHeight; j < maxHeight; j++)
			{
				int lineStride = imgWidth*j;
				for (int i = 0; i < maxWidth; i++)
				{
					int idx = lineStride + i;
					tmp2Data[idx] = ThreeBandPackedUtil.getPackedPixel(tmp2Data[idx]*255/valMax, tmp2Data[idx]*255/valMax, tmp2Data[idx]*255/valMax);
				}
			}
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmp2Data, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_2_2.bmp");
		}
		//*/

		//////////
		// Procura pelos maiores valores.
		/*
			
		*/
		int inclCurrentLineStride = 0;
		int [] stageInputData = tmp2Data;
		int maxHeight = imgHeight - wndHeight, maxWidth = imgWidth - 5;
		for (int j = wndHeight/2 + 1; j < maxHeight; j += wndHeight)
		{
			int lineStride = imgWidth*j;
			for (int i = 0; i < maxWidth; i++)
			{
				int idx = lineStride + i;
				int val = stageInputData[idx];
				int mask = maskData[idx];
				if (mask == 0 && val > tresholdHigher)
				{
					// Passo: procura o máximo por 4 posições
					{
						int iLeft = i;
						int iRight = i + 4; //if (iRight == imgWidth) iRight = imgWidth - 1;
						int valTmp = val, iMaxTmp = i;
						boolean bNotValid = false;
						while (iLeft < iRight + 1)
						{
							int idxTmp = lineStride + iLeft;
							if (maskData[idxTmp] == 0)
							{
								maskData[idxTmp] |= VISITED;
								if (stageInputData[idxTmp] >= valTmp)
									{valTmp = stageInputData[idxTmp]; iMaxTmp = iLeft;}
							}
							else
							{
								bNotValid = true;
								break;
							}
							iLeft++;
						}

						// Se pode ter um máximo fora da zona de busca ...
						if (iMaxTmp == iRight)
						{
							iMaxTmp = maxWidth - 1;
							while (iLeft < maxWidth)
							{
								int idxTmp = lineStride + iLeft;
								int idxTmpPrev = lineStride + iLeft - 1;
								if (maskData[idxTmp] == 0)
								{
									maskData[idxTmp] |= VISITED_UNKNOWN;
									if (stageInputData[idxTmp] < stageInputData[idxTmpPrev])
									{
										iMaxTmp = iLeft - 1;

										// marca como já visto elementos onde se sabe
										// que não se encontrará outra reta
										do
										{
											maskData[idxTmp] = VISITED_UNKNOWN_2;
											iLeft++;
											idxTmp = lineStride + iLeft;
											idxTmpPrev = lineStride + iLeft - 1;
										}
										while (iLeft < maxWidth && maskData[idxTmp] == 0 && stageInputData[idxTmp] < stageInputData[idxTmpPrev]);
										break;
									}
								}
								else
								{
									bNotValid = true;
									break;
								}
								iLeft++;
							}
						}
						if (!bNotValid)
						{
							i = iMaxTmp + 1;
							maskData[lineStride + i] = LINE_CENTER;
						}
						else
						{
							// Se estiver próximo de um pixel já visitado e usado, vai em frente.
							i = iLeft + 1;
							continue;
						}
					}

					inclTmpData[inclCurrentLineStride] = i; inclTmpData[inclCurrentLineStride + 1] = j;
					inclTmpData[inclCurrentLineStride + 2] = i;
					int nextInclPos = inclCurrentLineStride + 3;

					int iBottom = 0, jBottom = 0, iTop = 0, jTop = 0;
					int iLeftmost = i, iRightmost = i;

					// Passo: desce até encontrar limite de treshold inferior.
					int iCenter = i, jCenter = j + 1;
					int blindTries = 0;
					while (jCenter < imgHeight)
					{
						int centerLinestride = imgWidth*jCenter;
						int iLeftLeft = iCenter - 2, iLeft = iCenter - 1;  if (iLeftLeft < 0)	{iLeftLeft = 0; iLeft = 0;}
						int iRightRight = iCenter + 2, iRight = iCenter + 1;  if (iRightRight > imgWidth - 1)	{iRight = imgWidth - 1; iRightRight = imgWidth - 1;}
						int maxOfThree = tresholdLower, idxMaxOfThree = -1, dirValue = maskData[centerLinestride + iCenter];
						if (dirValue >= maxOfThree)
							{maxOfThree = dirValue + tresholdChangeDir; idxMaxOfThree = iCenter;}
						while (iLeftLeft < iLeft)
							maskData[centerLinestride + iLeftLeft++] |= VISITED;
						int maskAround = 0;
						while (iLeft < iRight + 1)
						{
							int idxTmp = centerLinestride + iLeft;
							if (stageInputData[idxTmp] > maxOfThree)
								{maxOfThree = stageInputData[idxTmp]; idxMaxOfThree = iLeft;}
							maskAround |= maskData[idxTmp];
							maskData[idxTmp] |= VISITED;
							iLeft++;
						}
						while (iRight < iRightRight)
							maskData[centerLinestride + iRight++] |= VISITED;

						if (idxMaxOfThree == -1)
						{
							blindTries++;
							if (blindTries == maxBlindTries)
							{
								jCenter -= (blindTries - 1);
								nextInclPos -= (blindTries - 1);
								blindTries = 0;
								break;
							}
							else
							{
								maskData[centerLinestride + iCenter] = LINE_PATH;
								
								/*PRINTIMGS
								maxOutData[centerLinestride + iCenter] = 0xFF0000;
								//*/
							}
						}
						else
						{
							blindTries = 0;
							if (maskAround != 0)
							{
								maskData[lineStride + i] |= REJECTED_TOO_CLOSE;
								
								/*PRINTIMGS
								maxOutData[lineStride + i] = 0x00FF00;
								maxOutData[centerLinestride + iCenter] = 0x7F7F7F;
								//*/

								jCenter++;
								break;
							}
							else
							{
								iCenter = idxMaxOfThree;
								maskData[centerLinestride + iCenter] = LINE_PATH;
								if (iCenter > iRightmost)
									iRightmost = iCenter;
								else if (iCenter < iLeftmost)
									iLeftmost = iCenter;
								
								/*PRINTIMGS
								maxOutData[centerLinestride + iCenter] = 0x0000FF;
								//*/
							}
						}
						inclTmpData[nextInclPos - 1] = iCenter - inclTmpData[nextInclPos - 1];
						inclTmpData[nextInclPos] = iCenter;
						nextInclPos++;
						jCenter++;
					}
					if (jCenter == imgHeight)
					{
							nextInclPos -= blindTries;
							jCenter -= blindTries;
					}
					inclTmpData[nextInclPos - 1] = 0xFF;
					iBottom = iCenter; jBottom = jCenter - 1;
//					log.debug("iBottom = " + iBottom + ", jBottom = " + jBottom + ", inclTmpData[nextInclPos - 1] = 0xFF: nextInclPos = " + nextInclPos);

					// Passo: sobe até encontrar limite de treshold inferior.
					nextInclPos = inclCurrentLineStride + imgWidth - 1;
					iCenter = i; jCenter = j - 1;
					inclTmpData[nextInclPos] = iCenter;
					nextInclPos--;
					blindTries = 0;
					while (jCenter > -1)
					{
						int centerLinestride = imgWidth*jCenter;
						int iLeftLeft = iCenter - 2, iLeft = iCenter - 1; if (iLeftLeft < 0)	{iLeftLeft = 0; iLeft = 0;}
						int iRightRight = iCenter + 2, iRight = iCenter + 1; if (iRightRight > imgWidth - 1)	{iRight = imgWidth - 1; iRightRight = imgWidth - 1;}
						int maxOfThree = tresholdLower, idxMaxOfThree = -1, dirValue = maskData[centerLinestride + iCenter];
						if (dirValue >= maxOfThree)
							{maxOfThree = dirValue + tresholdChangeDir; idxMaxOfThree = iCenter;}
						while (iLeftLeft < iLeft)
							maskData[centerLinestride + iLeftLeft++] |= VISITED;
						int maskAround = 0;
						while (iLeft < iRight + 1)
						{
							int idxTmp = centerLinestride + iLeft;
							if (stageInputData[idxTmp] > maxOfThree)
								{maxOfThree = stageInputData[idxTmp]; idxMaxOfThree = iLeft;}
							maskAround |= maskData[idxTmp];
							maskData[idxTmp] |= VISITED;
							iLeft++;
						}
						while (iRight < iRightRight)
							maskData[centerLinestride + iRight++] |= VISITED;

						if (idxMaxOfThree == -1)
						{
							blindTries++;
							if (blindTries == maxBlindTries)
							{
								jCenter += (blindTries - 1);
								nextInclPos += (blindTries - 1);
								blindTries = 0;
								break;
							}
							else
							{
								maskData[centerLinestride + iCenter] = LINE_PATH;

								/*PRINTIMGS
								maxOutData[centerLinestride + iCenter] = 0xFF0000;
								//*/
							}
						}
						else
						{
							blindTries = 0;
							if (maskAround != 0)
							{
								maskData[lineStride + i] |= REJECTED_TOO_CLOSE;

								/*PRINTIMGS
								maxOutData[lineStride + i] = 0x00FF00;
								maxOutData[centerLinestride + iCenter] = 0x7F7F7F;
								//*/

								jCenter--;
								break;
							}
							else
							{
								iCenter = idxMaxOfThree;
								maskData[centerLinestride + iCenter] = LINE_PATH;
								if (iCenter > iRightmost)
									iRightmost = iCenter;
								else if (iCenter < iLeftmost)
									iLeftmost = iCenter;

								/*PRINTIMGS
								maxOutData[centerLinestride + iCenter] = 0x0000FF;
								//*/
							}
						}
						inclTmpData[nextInclPos + 1] = inclTmpData[nextInclPos + 1] - iCenter;
						inclTmpData[nextInclPos] = iCenter;
						jCenter--;
						nextInclPos--;
					}
					if (jCenter == -1)
					{
							nextInclPos += blindTries;
							jCenter += blindTries;
					}
					inclTmpData[nextInclPos + 1] = 0xFF;
					iTop = iCenter; jTop = jCenter + 1;

					// Passo: Verifica se trecho é válido como linha: se é grande
					// o suficiente e se não é delgado.
					int lineLength = jBottom - jTop;
					if (lineLength > minSegmentLength)
					{
						inclTmpData[inclCurrentLineStride] = iTop; 
						inclTmpData[inclCurrentLineStride + 1] = jTop;

						inclCurrentLineStride += imgWidth;

						maskData[lineStride + i] = LINE_CENTER;
					}
					else
					{
						inclTmpData[inclCurrentLineStride] = 0xFF; 
						inclTmpData[inclCurrentLineStride + 1] = 0xFF;

						maskData[lineStride + i] = REJECTED_OUT_OF_SPEC;
						
						/*PRINTIMGS
						maxOutData[lineStride + i] = 0xFF00FF;
						//*/
					}
					i += 2;
				}
			}
		}

		/////////
		// Coloca a matriz de inclinações-de-dois numa forma mais útil.
		//
		inclCurrentLineStride = 0;
		int inclCurrentLineStrideNew = 0;
		for (int row = 0; row < imgHeight && inclTmpData[inclCurrentLineStride] != 0xFFFF; row++)
		{
			if (inclTmpData[inclCurrentLineStride] != 0xFF)
			{
				int colNew = 0;
				int j = imgWidth - 1; for (; inclTmpData[inclCurrentLineStride + j] != 0xFF; j--);

//				log.debug("imgWidth = " + imgWidth + ", j = " + j);
				j++; for (; j < imgWidth; j++)
					inclData[inclCurrentLineStrideNew + colNew++] = inclTmpData[inclCurrentLineStride + j];
				j = 2; for (; inclTmpData[inclCurrentLineStride + j] != 0xFF; j++)
					inclData[inclCurrentLineStrideNew + colNew++] = inclTmpData[inclCurrentLineStride + j];

//				log.debug("j after = " + j);
				inclData[inclCurrentLineStrideNew + colNew] = 0xFF;
				inclData[inclCurrentLineStrideNew + imgWidth - 1] = inclTmpData[inclCurrentLineStride];
				inclData[inclCurrentLineStrideNew + imgWidth - 2] = inclTmpData[inclCurrentLineStride + 1];
				inclData[inclCurrentLineStrideNew + imgWidth - 3] = colNew;
//				log.debug("iTop = " + inclTmpData[inclCurrentLineStride] + ", jTop = " + inclTmpData[inclCurrentLineStride + 1] + ", width = " + colNew);
				inclCurrentLineStrideNew += imgWidth;
			}
			inclCurrentLineStride += imgWidth;
		}
		inclData[inclCurrentLineStrideNew] = 0xFFFF;


		/*PRINTIMGS
		if (bGenerateOutput)
		{
			inclCurrentLineStride = 0;
			java.util.Arrays.fill(tmpOutData, 0xFFFF);
			for (int i = 0; i < imgHeight && inclData[inclCurrentLineStride] != 0xFFFF; i++)
			{
				int iCenter = inclData[inclCurrentLineStride + imgWidth - 1], jCenter = inclData[inclCurrentLineStride + imgWidth - 2];
				for (int j = 0; inclData[inclCurrentLineStride + j] != 0xFF; j++)
				{
					int stride = jCenter*imgWidth;
					tmpOutData[stride + iCenter] = inclData[inclCurrentLineStride + j];
					jCenter++;
					stride = jCenter*imgWidth;
					if ((maskData[stride + iCenter] & LINE_PATH) != 0)
						iCenter = iCenter;
					else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
						iCenter = iCenter + 1;
					else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
						iCenter = iCenter - 1;
					else
						{log.debug("no no no 2! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): j = " + j); break;}
				}
				inclCurrentLineStride += imgWidth;
			}
			lookupFilter.setColorMap(arraColorInclSoftIn, arraColorInclSoftOut, 0x00009F);
			lookupFilter.operate(tmpOutData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_3_1.bmp");
		}
		//*/

		{
			inclCurrentLineStride = 0;
			for (int row = 0; inclData[inclCurrentLineStride] != 0xFFFF; row++)
			{
				int sizeLine = inclData[inclCurrentLineStride + imgWidth - 3];
//				if (sizeLine > 4) for (int col = 0; col < sizeLine && inclData[inclCurrentLineStride + col + 4] != 0xFF;)
				if (sizeLine > 4) for (int col = 0; col < sizeLine;)
				{
					int firstVal = inclData[inclCurrentLineStride + col];
					if (firstVal == 0)
					{
						int lastVal1 = inclData[inclCurrentLineStride + col + 3];
						int sumVal1 = 0;
						if (lastVal1 == firstVal)
						{
							sumVal1 = inclData[inclCurrentLineStride + col + 1] +
									  inclData[inclCurrentLineStride + col + 2];
							if (sumVal1 == 0)
							{
								inclData[inclCurrentLineStride + col + 1] = lastVal1;
								inclData[inclCurrentLineStride + col + 2] = lastVal1;
								col += 3;
							}
							else
								col++;
						}
						else
						{
							int lastVal2 = inclData[inclCurrentLineStride + col + 4];
							if (lastVal2 == firstVal)
							{
								int sumVal2 = sumVal1 + inclData[inclCurrentLineStride + col + 3];
								if (sumVal2 == 0)
								{
									inclData[inclCurrentLineStride + col + 1] = lastVal2;
									inclData[inclCurrentLineStride + col + 2] = lastVal2;
									inclData[inclCurrentLineStride + col + 3] = lastVal2;
									col += 4;
								}
								else
									col++;
							}
							else
								col++;
						}
					}
					else
					{
						int nextVal = inclData[inclCurrentLineStride + col + 1];
						if ((nextVal + firstVal) == 0)
						{
							inclData[inclCurrentLineStride + col] = 0;
							inclData[inclCurrentLineStride + col + 1] = 0;
							col += 2;
						}
						else
							col++;
					}
				}
				inclCurrentLineStride += imgWidth;
			}
		}
		
		/////////
		// Cria a matriz de inclinações-de-dois suavizada, onde cada entrada tenta representar a
		// inclinação média das redondezas do pixel no rastro.
		//
		int cutoff = 4;
		{
			inclCurrentLineStride = 0;
			for (int row = 0; inclData[inclCurrentLineStride] != 0xFFFF; row++)
			{
				inclSoftData[inclCurrentLineStride + 0] = 0xFF; inclSoftData[inclCurrentLineStride + 1] = 0xFF;
				inclSoftData[inclCurrentLineStride + 2] = 0xFF; inclSoftData[inclCurrentLineStride + 3] = 0xFF;

				int sizeLine = inclData[inclCurrentLineStride + imgWidth - cutoff];
				int col = cutoff;
				if (sizeLine > 10 && sizeLine > minSegmentLength) 
				{
					for (; inclData[inclCurrentLineStride + col + cutoff] != 0xFF; col++)
					{
						inclSoftData[inclCurrentLineStride + col] = inclData[inclCurrentLineStride + col - 4] + 
																	inclData[inclCurrentLineStride + col - 3] + 
																	inclData[inclCurrentLineStride + col - 2] + 
																	inclData[inclCurrentLineStride + col - 1] + 
																	inclData[inclCurrentLineStride + col] + 
																	inclData[inclCurrentLineStride + col + 1] + 
																	inclData[inclCurrentLineStride + col + 2] + 
																	inclData[inclCurrentLineStride + col + 3] + 
																	inclData[inclCurrentLineStride + col + 4];
//						log.debug("inclSoftData[" + inclCurrentLineStride/imgWidth + "]["+col+"]: " + inclSoftData[inclCurrentLineStride + col]);
					}
					int iCenter = inclData[inclCurrentLineStride + imgWidth - 1], jCenter = inclData[inclCurrentLineStride + imgWidth - 2];
					for (int count = 0; count < 3; count++)
					{
						jCenter++;
						int stride = jCenter*imgWidth;
						if ((maskData[stride + iCenter] & LINE_PATH) != 0)
							iCenter = iCenter;
						else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
							iCenter = iCenter + 1;
						else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
							iCenter = iCenter - 1;
						else
							{log.debug("no no no 2.5! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): count = " + count); break;}
					}
					inclSoftData[inclCurrentLineStride + imgWidth - 1] = iCenter;
					inclSoftData[inclCurrentLineStride + imgWidth - 2] = jCenter;
					inclSoftData[inclCurrentLineStride + imgWidth - 3] = inclData[inclCurrentLineStrideNew + imgWidth - 3] - cutoff*2;

/*
					long iMidle = 0;
					int countSize = 0;
					for (int j = 4; inclSoftData[inclCurrentLineStride + j] != 0xFF; j++)
					{
						iMidle += iCenter; countSize++;

						int stride = jCenter*imgWidth;
						jCenter++;
						stride = jCenter*imgWidth;
						if ((maskData[stride + iCenter] & LINE_PATH) != 0)
							iCenter = iCenter;
						else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
							iCenter = iCenter + 1;
						else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
							iCenter = iCenter - 1;
						else
							{log.debug("no no no 2222! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): j = " + j); break;}
					}
					inclSoftData[inclCurrentLineStride + imgWidth - 4] = iMidle/countSize;
//*/
/*
					log.debug("inclSoftData[" + inclCurrentLineStride/imgWidth + "][imgWidth - 1]: " + iCenter);
					log.debug("inclSoftData[" + inclCurrentLineStride/imgWidth + "][imgWidth - 2]: " + jCenter);
					log.debug("inclSoftData[" + inclCurrentLineStride/imgWidth + "][imgWidth - 3]: " + (inclData[inclCurrentLineStrideNew + imgWidth - 3] - cutoff*2));
					log.debug("inclSoftData[" + inclCurrentLineStride/imgWidth + "][imgWidth - 4]: " + iMidle);
//*/
				}
				inclSoftData[inclCurrentLineStride + col] = 0xFF;
//				log.debug("[end]inclSoftData[" + inclCurrentLineStride/imgWidth + "]["+col+"]: " + inclSoftData[inclCurrentLineStride + col]);

				inclCurrentLineStride += imgWidth;
			}
			inclSoftData[inclCurrentLineStride] = 0xFFFF;
//			log.debug("[endend]inclSoftData[" + inclCurrentLineStride/imgWidth + "][0]: " + inclSoftData[inclCurrentLineStride]);
		}
		
		/*PRINTIMGS
		if (bGenerateOutput)
		{
			br.com.r4j.commons.util.Arrays.arrayCopy(inData, tmpOut3Data);

			inclCurrentLineStride = 0;
			java.util.Arrays.fill(tmpOutData, 0xFFFF);
			for (int i = 0; i < imgHeight && inclSoftData[inclCurrentLineStride] != 0xFFFF; i++)
			{
				int iCenter = inclSoftData[inclCurrentLineStride + imgWidth - 1], jCenter = inclSoftData[inclCurrentLineStride + imgWidth - 2];
				for (int j = cutoff; inclSoftData[inclCurrentLineStride + j] != 0xFF; j++)
				{
					int stride = jCenter*imgWidth;
					tmpOutData[stride + iCenter] = inclSoftData[inclCurrentLineStride + j];
					tmpOut3Data[stride + iCenter] = 0xFFFFFF;
					jCenter++;
					stride = jCenter*imgWidth;
					if ((maskData[stride + iCenter] & LINE_PATH) != 0)
						iCenter = iCenter;
					else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
						iCenter = iCenter + 1;
					else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
						iCenter = iCenter - 1;
					else
						{log.debug("no no no 2! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): j = " + j); break;}
				}
				inclCurrentLineStride += imgWidth;
			}
			lookupFilter.setColorMap(arraColorInclSoftIn, arraColorInclSoftOut, 0x00009F);
			lookupFilter.operate(tmpOutData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_4_1_1.bmp");
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOut3Data, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_4_1_2.bmp");
		}
		//*/


		/*PRINTIMGS
		if (bGenerateOutput)
		{
//			histOp.operate(tmpData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(maxOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_5.bmp");
			bitMaskLookupFilter.setColorMap(arraColorMaskIn, arraColorMaskOut, 0x00007F);
			bitMaskLookupFilter.operate(maskData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_6.bmp");
			lookupFilter.setColorMap(arraColorInclIn, arraColorInclOut, 0x00007F);
			lookupFilter.operate(inclData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_7_1.bmp");
			lookupFilter.setColorMap(arraColorInclSoftIn, arraColorInclSoftOut, 0x0000FF);
			lookupFilter.operate(inclSoftData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_7_2.bmp");

			inclCurrentLineStride = 0;
			java.util.Arrays.fill(tmpOutData, 0xFFFF);
			for (int i = 0; i < imgHeight && inclData[inclCurrentLineStride] != 0xFFFF; i++)
			{
				int iCenter = inclData[inclCurrentLineStride + imgWidth - 1], jCenter = inclData[inclCurrentLineStride + imgWidth - 2];
//				log.debug("iCenter = " + iCenter + ", jCenter = " + jCenter + ", size = " + inclData[inclCurrentLineStride + imgWidth - 3]);
				for (int j = 0; inclData[inclCurrentLineStride + j] != 0xFF; j++)
				{
					int stride = jCenter*imgWidth;
					tmpOutData[stride + iCenter] = inclData[inclCurrentLineStride + j];
					jCenter++;
					stride = jCenter*imgWidth;
					if ((maskData[stride + iCenter] & LINE_PATH) != 0)
						iCenter = iCenter;
					else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
						iCenter = iCenter + 1;
					else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
						iCenter = iCenter - 1;
					else
						{log.debug("no no no 1! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): i = " + i + ", maskData[stride + iCenter] = " + maskData[stride + iCenter] + ", iCenter = " + iCenter + ", jCenter = " + jCenter);}
				}
				inclCurrentLineStride += imgWidth;
			}
			lookupFilter.setColorMap(arraColorInclIn, arraColorInclOut, 0x007F7F);
			lookupFilter.operate(tmpOutData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_8_2.bmp");

			inclCurrentLineStride = 0;
			java.util.Arrays.fill(tmpOutData, 0xFFFF);
			for (int i = 0; i < imgHeight && inclSoftData[inclCurrentLineStride] != 0xFFFF; i++)
			{
				int iCenter = inclSoftData[inclCurrentLineStride + imgWidth - 1], jCenter = inclSoftData[inclCurrentLineStride + imgWidth - 2];
				for (int j = 4; inclSoftData[inclCurrentLineStride + j] != 0xFF; j++)
				{
					int stride = jCenter*imgWidth;
					tmpOutData[stride + iCenter] = inclSoftData[inclCurrentLineStride + j];
					jCenter++;
					stride = jCenter*imgWidth;
					if ((maskData[stride + iCenter] & LINE_PATH) != 0)
						iCenter = iCenter;
					else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
						iCenter = iCenter + 1;
					else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
						iCenter = iCenter - 1;
					else
						{log.debug("no no no 2! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): j = " + j); break;}
				}
				inclCurrentLineStride += imgWidth;
			}
			lookupFilter.setColorMap(arraColorInclSoftIn, arraColorInclSoftOut, 0x00009F);
			lookupFilter.operate(tmpOutData, tmpOutData, imgWidth, imgHeight);
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(tmpOutData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), (itCount) + "_step_9_2.bmp");
		}
		//*/

		/////////
		// Cria representação (valor, tamanho).
		//
		// Representação formada por um vetor com: (x, y, x_end, y_end, valor, tamanho).
		//
		{
			inclCurrentLineStride = 0;
			int posCount = 0;
			int row = 0; for (;inclSoftData[inclCurrentLineStride] != 0xFFFF; row++)
			{
				int valAnt = 0xFFFFFF;
				int iCenter = inclSoftData[inclCurrentLineStride + imgWidth - 1];
				int jCenter = inclSoftData[inclCurrentLineStride + imgWidth - 2];
//				int iMidle = inclSoftData[inclCurrentLineStride + imgWidth - 4];
				int iCenterLast = -1;
				int sizeSeg = 0, iSeg = iCenter, jSeg = jCenter;
				int stride = jCenter*imgWidth;
				int posCountIni = posCount;
				posCount++;
				int countChanges = 0;
				for (int col = 4; inclSoftData[inclCurrentLineStride + col] != 0xFF; col++)
				{
					while (inclSoftData[inclCurrentLineStride + col] != 0xFF && col < 6) col++;

					int val = inclSoftData[inclCurrentLineStride + col];
					if (valAnt != val)
					{
						if (valAnt != 0xFFFFFF)
						{
							inclData[posCount + X_FIRST] = iSeg;
							inclData[posCount + Y_FIRST] = jSeg;
							inclData[posCount + X_LAST] = iCenterLast;
							inclData[posCount + Y_LAST] = jCenter - 1;
							inclData[posCount + VAL] = valAnt;
							inclData[posCount + LENGTH] = sizeSeg;
							posCount += JUMP;

							iSeg = iCenter; jSeg = jCenter;
							countChanges++;
						}
						sizeSeg = 1;
						valAnt = val;
					}
					else
						sizeSeg++;

					stride += imgWidth;
					jCenter++;
					iCenterLast = iCenter;
					if ((maskData[stride + iCenter] & LINE_PATH) != 0)
						iCenter = iCenter;
					else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
						iCenter = iCenter + 1;
					else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
						iCenter = iCenter - 1;
					else
						{log.debug("no no no 3! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): col = " + col); break;}
				}
				inclData[posCount + X_FIRST] = iSeg;
				inclData[posCount + Y_FIRST] = jSeg;
				inclData[posCount + X_LAST] = iCenterLast;
				inclData[posCount + Y_LAST] = jCenter - 1;
				inclData[posCount + VAL] = valAnt;
				inclData[posCount + LENGTH] = sizeSeg;
				posCount += JUMP;

				inclData[posCount] = 0xFF;
				inclData[posCountIni] = countChanges + 1;

				inclCurrentLineStride += imgWidth;
				posCount++;
			}
			inclData[inclCurrentLineStride] = 0xFFFF;
		}


		/////////
		// Segmenta do seguinte modo: 
		//
		//  - pega o primeiro valor.
		//  - pega o segundo valor.
		//  - anda na linha até encontrar um terceiro valor.
		//  - anda na linha enquanto alternar entre o segundo e terceiro valor.
		//  - verifica o que é maior: a coleção de primeiro e segundo valores, ou a de segundo e terceiro valores.
		//  - se for de primeiro e segundo:
		//     -- cria segmento de primeiro e segundo com o último segundo fazendo parte desse segmento.
		//  - se for de segundo e terceiro:
		//     -- cria segmento de primeiro e segundo com o último segundo não fazendo parte desse segmento.
		//   - se o segmento de primeiro e segundo for maior que X, considera válido para próxmia etapa.
		//   - chama o segundo valor de primeiro, o terceiro de segundo e o novo de tercei
		//   - repete até não ter mais.
		//
		// Representação:
		//
		// - segmento: inclinação, ponto em que linha infinita cruza no centro da imagem (só coord X), tamanho, ponto inicial do segmento(x e y) (um objeto para cada: LineSegment).
		//
		// - armazenamento: ordenado pela coord X, pela inclinação e pelo centro.
		//
		TreeSet setOrder = new TreeSet();
		{
			int actualJump = 0;
			while (inclData[actualJump] < 0xFF)
			{
				int numSegsFixo = inclData[actualJump]; 
				int numSegs = numSegsFixo; actualJump++;

				/*PRINTIMGS
				log.debug("-- numSegs = " + numSegs);
				//*/

				int sizesinho = 7;
				int [] valsUsed = new int[sizesinho], accVals = new int[sizesinho], vals = new int[sizesinho];
				int posCount = 0, valsIdx = 0, countVals = 0, posIni = 0, valsIdxCenter = sizesinho/2;
				boolean bLook4FirstSeg = true, bHasSeg = false;
				valsIdx = valsIdxCenter;
				if (numSegs > 0) while (true)
				{
					if (numSegs > 0)
					{
						int val = inclData[actualJump + posCount + VAL];

						if (valsUsed[valsIdx] == 0)
						{
							valsUsed[valsIdx] = 1;
							accVals[valsIdx] += inclData[actualJump + posCount + LENGTH];
							vals[valsIdx] = val;
						}
						else
						{
							int valPrev = vals[valsIdx];

							// Diferença consecutiva maior que 2
							if (Math.abs(valPrev - val) > 3)
								bHasSeg = true;
							else
							{
								valsIdx += (val - valPrev);

								// Diferença total maior que 2 (no máximo três valores)
								if (Math.abs(valsIdx - valsIdxCenter) > valsIdxCenter)
									bHasSeg = true;

								// Ok, apenas um valor novo. Ou um repetido.
								else
								{
									valsUsed[valsIdx] = 1;
									accVals[valsIdx] += inclData[actualJump + posCount + LENGTH];
									vals[valsIdx] = val;
								}
							}
						}
					}
					else
						bHasSeg = true;

					if (bHasSeg)
					{
						/*PRINTIMGS
						String strSegInc1 = "", strSegInc2 = "", strSegInc3 = "";
						//*/

						double meanM = 0;
						int sizeSeg = 0;

						for (int i = 0; i < valsUsed.length; i++)
						{
							if (valsUsed[i] != 0)
							{
								/*PRINTIMGS
								strSegInc1 += vals[i] + ", ";
								strSegInc2 += accVals[i] + ", ";
								strSegInc3 += (accVals[i]*vals[i]) + ", ";
								//*/
								meanM += ((double) accVals[i]*vals[i]);
								sizeSeg += accVals[i];

							}
							valsUsed[i] = 0;
							accVals[i] = 0;
						}
						valsIdx = valsIdxCenter;
						// A divisão depende do tamanho escolhido para fazer a suavização.
						meanM = meanM/9.0/sizeSeg;

						if (sizeSeg > minSegmentLength && Math.abs(meanM) < maxM*1.8)
						{
							LineSegment lineSeg = this.createSegment(weightData, maskData, inclData, actualJump, meanM, posIni, posCount - JUMP, imgWidth, imgHeight);
							if (lineSeg.xMidMean > 3 && lineSeg.xMidMean < 317)
								setOrder.add(new LineSegHolder(lineSeg));
						}

						countVals = 0;
						posIni = posCount;

						/*PRINTIMGS
//*
						log.debug("inclinacoes 1: " + strSegInc1);
						log.debug("inclinacoes 2: " + strSegInc2);
						log.debug("inclinacoes 3: " + strSegInc3);
						log.debug("meanM: " + meanM);
//*/
						//*/

						if (numSegs <= 0)
							break;
						else
							bHasSeg = false;
					}
					else
					{
						posCount += JUMP;
						numSegs--;
					}
				}
				else
				{
					if (log.isDebugEnabled())
						log.debug("numSegs <= 0 !!!!!!!!!: " + numSegs);
					break;
				}
				actualJump += numSegsFixo*JUMP + 1;
			}

			/*PRINTIMGS
			if (bGenerateOutput)
				br.com.r4j.commons.util.Arrays.arrayCopy(inData, arrayOutDebug);
			log.debug("setOrder.size() = " + setOrder.size());
			Iterator itDebug = setOrder.iterator();
			while (itDebug.hasNext())
			{
				LineSegHolder lineSegHolder = (LineSegHolder) itDebug.next();
				LineSegment lineSeg = lineSegHolder.lineSeg;
				log.debug("-1-lineSeg = " + lineSeg);

				if (bGenerateOutput)
				{
					Color clr = new Color(255, 0, 0);
					if (lineSeg.mModel < 0)
						clr = new Color(0, 0, 255);
					else if (lineSeg.mModel == 0)
						clr = new Color(0, 0, 0);
					int clrVal = ThreeBandPackedUtil.getPackedPixel(clr);
					ThreeBandPackedUtil.plotLineInvModel((float) lineSeg.mModel, (int) lineSeg.xModel, 
														lineSeg.xIni, lineSeg.yIni,  lineSeg.xIni + lineSeg.sizeSeg, lineSeg.yIni + lineSeg.sizeSeg, 
														clrVal, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.fillRect(lineSeg.xIni - 1, lineSeg.yIni - 1, 2, 2, 0x00FF00, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.fillRect(lineSeg.xEnd - 1, lineSeg.yEnd - 1, 2, 2, 0xFF0000, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.plotLine(lineSeg.xIni, lineSeg.yIni,  lineSeg.xEnd, lineSeg.yEnd, 
														0x0000FF, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.plotString(lineSeg.xEnd + 5, lineSeg.yEnd + 15, 
														0xFF00FF, "" + ((lineSeg.xEnd + lineSeg.xIni)/2), arrayOutDebug, imgWidth, imgHeight);
				}
			}
			if (bGenerateOutput)
				ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(arrayOutDebug, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), "debA_00_" + itCount + ".bmp");
			log.debug("setOrder.size() = " + setOrder.size());
			//*/
		}


		/////////
		// Gera as retas finais:
		//
		// Junta os segmentos que possuem coord X e incl muito parecidos -- iguais.
		//
		//
		// Representação:
		//
		// - incl e ponto em que cruza o centro da imagem, e tamamnho total.
		// - lista com todos os segmentos que compoõe a reta.
		//
		ArrayList listLines = new ArrayList();
		TreeSet setSegColls = new TreeSet();

		LineSegmentCollection lastSegmentCol = null;

		if (setOrder.size() > 0) for (float diffLim = (float) 0.1; diffLim < (float) 2.0;)
		{
			TreeSet setPairs = new TreeSet();
			Iterator itSegs = setOrder.iterator();
			LineSegHolder segH = (LineSegHolder) itSegs.next();
			LineSegHolder lastSegH = null;
			while (itSegs.hasNext())
			{
				lastSegH = segH;
				segH = (LineSegHolder) itSegs.next();

				LineSegment seg = segH.lineSeg;
				LineSegment lastSeg = lastSegH.lineSeg;
				if (lastSeg.superimpose(seg))
				{
/*PRINTIMGS
					if (log.isDebugEnabled())
						log.debug("superimpose!:seg: " + seg + ", lastSeg: " + lastSeg);
//*/
					continue;
				}

				double diff = (((double) ((int) (Math.abs(seg.xMidle - lastSeg.xMidle)*10)))/10.0);
/*PRINTIMGS
				if (log.isDebugEnabled())
					log.debug("diff: " + diff);
//*/
				if (diff <= diffLim)
				{
					double diffM = Math.abs(Math.atan(seg.mModel) - Math.atan(lastSeg.mModel));
/*PRINTIMGS
					log.debug("diffM: " + diffM);
//*/
//					if (diffM <= (1.8 + 3*diffLim)*Math.PI/180)
					if (diffM <= 2.5*Math.PI/180)
					{
/*PRINTIMGS
						if (log.isDebugEnabled())
							log.debug("aproved:seg: " + seg + ", lastSeg: " + lastSeg);
//*/
						if (lastSeg.getClass() == LineSegmentCollection.class)
						{
							LineSegPairs pair = new LineSegPairs(lastSegH, segH);
							setPairs.add(pair);
						}
						else
						{
							LineSegPairs pair = new LineSegPairs(segH, lastSegH);
							setPairs.add(pair);
						}
					}
					else
					{
/*PRINTIMGS
						if (log.isDebugEnabled())
							log.debug("blocked by M!:seg: " + seg + ", lastSeg: " + lastSeg);
//*/
					}
				}
			}
/*PRINTIMGS
			if (log.isDebugEnabled())
				log.debug("setPairs.size(): " + setPairs.size());
//*/
			if (setPairs.size() > 0)
			{
				ArrayList listNewColls = new ArrayList();

				Iterator itPairs = setPairs.iterator();
				while (itPairs.hasNext())
				{
					LineSegPairs pair = (LineSegPairs) itPairs.next();
					LineSegmentCollection lsColl = null;
					if (pair.lineSeg1 == null || pair.lineSeg2 == null)
						continue;

					if (pair.lineSeg1.lineSeg.getClass() == LineSegmentCollection.class)
						lsColl = (LineSegmentCollection) pair.lineSeg1.lineSeg;
					else
						lsColl = new LineSegmentCollection(pair.lineSeg1.lineSeg);

					lsColl.addSeg(pair.lineSeg2.lineSeg);

					for (int cnt = 0; cnt < pair.lineSeg1.setPairs.size(); cnt++)
					{
						LineSegPairs otherPair = (LineSegPairs) pair.lineSeg1.setPairs.get(cnt);
						if (otherPair == pair) continue;
						otherPair.lineSeg1 = null; otherPair.lineSeg2 = null;
					}

					for (int cnt = 0; cnt < pair.lineSeg2.setPairs.size(); cnt++)
					{
						LineSegPairs otherPair = (LineSegPairs) pair.lineSeg2.setPairs.get(cnt);
						if (otherPair == pair) continue;
						otherPair.lineSeg1 = null; otherPair.lineSeg2 = null;
					}

					listNewColls.add(lsColl);
					if (log.isDebugEnabled())
						log.debug("removing: " + pair.lineSeg1 + ",   " + pair.lineSeg2);
					setOrder.remove(pair.lineSeg1);
					setOrder.remove(pair.lineSeg2);
				}
				Iterator itNewColls = listNewColls.iterator();
				while (itNewColls.hasNext())
				{
					LineSegmentCollection lsColl = (LineSegmentCollection) itNewColls.next();
					if (log.isDebugEnabled())
						log.debug("adding: " + lsColl);
					setOrder.add(new LineSegHolder(lsColl));
				}
			}
			else
			{
				diffLim += 0.5;
			}
		}

		/*PRINTIMGS
		log.debug("listLines.size() = " + listLines.size());
		if (bGenerateOutput)
			br.com.r4j.commons.util.Arrays.arrayCopy(inData, arrayOutDebug);
		//*/

		listLineParams.clear();
		Iterator itLines = setOrder.iterator();
		while (itLines.hasNext())
		{
			LineSegHolder lineSegHolder = (LineSegHolder) itLines.next();
			LineSegment lineSegCol = lineSegHolder.lineSeg;

			if (lineSegCol.sizeSeg > minLineLength && Math.abs(lineSegCol.mModel) < maxM)
			{
				/*PRINTIMGS
				log.debug("lineSegCol: " + lineSegCol);
				//*/

				if (lineSegCol.getClass() == LineSegmentCollection.class)
					listLineParams.add(lineSegCol);
				else
					listLineParams.add(new LineSegmentCollection(lineSegCol));

				/*PRINTIMGS
				if (bGenerateOutput)
				{
					Color clr = new Color(255, 255, 0);
					int clrVal = ThreeBandPackedUtil.getPackedPixel(clr);
					ThreeBandPackedUtil.plotLineInvModel((float) lineSegCol.mModel, (int) lineSegCol.xModel, 
														lineSegCol.xIni, lineSegCol.yIni,  lineSegCol.xEnd, lineSegCol.yEnd, 
														clrVal, arrayOutDebug, imgWidth, imgHeight);

					ThreeBandPackedUtil.fillRect(lineSegCol.xIni - 1, lineSegCol.yIni - 1, 2, 2, 0x00FF00, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.fillRect(lineSegCol.xEnd - 1, lineSegCol.yEnd - 1, 2, 2, 0xFF0000, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.plotLine(lineSegCol.xIni, lineSegCol.yIni,  lineSegCol.xEnd, lineSegCol.yEnd, 
													0x0000FF, arrayOutDebug, imgWidth, imgHeight);
					ThreeBandPackedUtil.plotString(lineSegCol.xEnd + 5, lineSegCol.yEnd + 5, 
														0xFF00FF, "" + ((lineSegCol.xEnd + lineSegCol.xIni)/2), arrayOutDebug, imgWidth, imgHeight);
				}
				//*/
			}
			/*PRINTIMGS
			else
				log.debug("rejected");
			//*/
		}

		/*PRINTIMGS
		if (bGenerateOutput)
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(arrayOutDebug,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "debA_01_" + itCount + ".bmp");
		//*/

		/*PRINTIMGS
		log.debug("listLineParams.size() = " + listLineParams.size());
		log.debug("fim da revisao");
		Iterator itDbg = listLineParams.iterator();
		while (itDbg.hasNext())
		{
//			log.debug("---> meas u: "+ ((LineSegmentCollection) itDbg.next()).getXMidleOfSegment());
			log.debug("---> meas u: "+ ((LineSegmentCollection) itDbg.next()).xMidMean2);
		}
		//*/


		/*PRINTIMGS
		if (bGenerateOutput)
		{
			for (int j = 0; j < imgHeight; j++)
			{
				int lineStride = imgWidth*j;
				for (int i = 0; i < imgWidth; i++)
				{
					int idx = lineStride + i;
					int mask = maskData[idx];
					if (mask == 1 || mask == 4)
					{
						int val = (mask == 1) ? 0xFFFF00 : 0x00FFFF;
						outData[idx] = val;

						int iCenter = i, jCenter = j;
						while (jCenter < imgHeight)
						{
							int centerLinestride = imgWidth*jCenter;
							int iLeft = i - 1; if (iLeft == -1) iLeft = 0; if (iLeft == imgWidth) iLeft = imgWidth - 1;
							int iRight = i + 1; if (iRight == -1) iRight = 0; if (iRight == imgWidth) iRight = imgWidth - 1;
							while (iLeft < iRight + 1)
							{
								if (maxOutData[centerLinestride + iLeft] == 0x0000FF)
								{
									if (jCenter == j)
										outData[centerLinestride + iLeft] = 0xFF0000;
									else
										outData[centerLinestride + iLeft] = val;
									break;
								}
								iLeft++;
							}
							if (iLeft == iRight + 1)
								break;
							else
								jCenter++;
						}

						if (imgWidth*jCenter + iCenter < max && imgWidth*jCenter + iCenter > -1)
							outData[imgWidth*jCenter + iCenter] = 0x00FF00;

						iCenter = i; jCenter = j - 1;
						while (jCenter > -1)
						{
							int centerLinestride = imgWidth*jCenter;
							int iLeft = i - 1; if (iLeft == -1) iLeft = 0; if (iLeft == imgWidth) iLeft = imgWidth - 1;
							int iRight = i + 1; if (iRight == -1) iRight = 0; if (iRight == imgWidth) iRight = imgWidth - 1;
							while (iLeft < iRight + 1)
							{
								if (maxOutData[centerLinestride + iLeft] == 0x0000FF)
								{
									outData[centerLinestride + iLeft] = val;
									break;
								}
								iLeft++;
							}
							if (iLeft == iRight + 1)
								break;
							else
								jCenter--;
						}
						if (imgWidth*jCenter + iCenter < max && imgWidth*jCenter + iCenter > -1)
							outData[imgWidth*jCenter + iCenter] = 0x00FF00;
					}
				}
			}
		}
		//*/
		return 0;
	}


	private LineSegment createSegment(int [] weightData, int [] maskData, int [] inclData, int jump, double meanM, int posBegin, int posLast, int imgWidth, int imgHeight)
	{
		LineSegment lineSeg = new LineSegment();
		lineSeg.imgHeight = imgHeight;

		int xBegin = inclData[jump + posBegin + X_FIRST];
		int yBegin = inclData[jump + posBegin + Y_FIRST];

		int xEnd = inclData[jump + posLast + X_LAST];
		int yEnd = inclData[jump + posLast + Y_LAST];

		/*PRINTIMGS
		String str = "seg("+xBegin+","+yBegin+","+xEnd+","+yEnd+")";
		log.debug(str);
		//*/

		lineSeg.sizeSeg = yEnd - yBegin + 1;

		double xMid1 = xBegin, yMid1 = yBegin;
		double xMid2 = xEnd, yMid2 = yEnd;

		if (posBegin != posLast)
		{
			xMid1 = ((double) (inclData[jump + posBegin + X_LAST] + xBegin))/2.0;
			yMid1 = ((double) (inclData[jump + posBegin + Y_LAST] + yBegin))/2.0;
			xMid2 = ((double) (inclData[jump + posLast + X_FIRST] + xEnd))/2.0;
			yMid2 = ((double) (inclData[jump + posLast + Y_FIRST] + yEnd))/2.0;
		}

		double xMid = (xMid1 + xMid2) / 2;
		double yMid = (yMid2 + yMid2) / 2;

		if (lineSeg.sizeSeg > 1)
		{
//			lineSeg.mModel = ((double) (xMid1 - xMid2)) / ((double) (yMid1 - yMid2));
//			lineSeg.mModel = ((double) (xEnd - xBegin)) / ((double) (yEnd - yBegin));
			lineSeg.mModel = meanM;
		}
		else
			lineSeg.mModel = 0;

		double b = ((xMid - lineSeg.mModel*yMid) + (xMid1 - lineSeg.mModel*yMid1) + (xMid2 - lineSeg.mModel*yMid2))/3;
		lineSeg.xModel = b;

		lineSeg.yIni = yBegin;
		lineSeg.yEnd = yEnd;
		lineSeg.xIni = (int) (xMid - lineSeg.mModel*lineSeg.sizeSeg/2);
		lineSeg.xEnd = (int) (xMid + lineSeg.mModel*(lineSeg.sizeSeg + 1)/2);


		int iCenter = xBegin;
		int jCenter = yBegin;
		int sumXMid = 0;
		double meanWXTot = 0;
		for (int j = 0; ; j++)
		{
			int stride = jCenter*imgWidth;

			double meanW = 0;
			double meanWX = 0;
			for (int idxW = -2; idxW < 3; idxW++)
			{
				meanW += weightData[stride + iCenter + idxW];
				meanWX += weightData[stride + iCenter + idxW]*(iCenter + idxW);
			}
			if (meanW > 0)
				meanWXTot += meanWX/meanW;
			else
			{
				if (log.isDebugEnabled())
					log.debug("meanW == 0: xBegin = " + xBegin + ", yBegin = " + yBegin + ", iCenter = " + iCenter + ", jCenter = " + jCenter + ", imgWidth = " + imgWidth);
			}

			sumXMid += iCenter;

			jCenter++;
			if (jCenter > yEnd)
				break;

			stride = jCenter*imgWidth;
			if ((maskData[stride + iCenter] & LINE_PATH) != 0)
				iCenter = iCenter;
			else if (iCenter < imgWidth - 1 && (maskData[stride + iCenter + 1] & LINE_PATH) != 0)
				iCenter = iCenter + 1;
			else if (iCenter > 0 && (maskData[stride + iCenter - 1] & LINE_PATH) != 0)
				iCenter = iCenter - 1;
			else
			{
				if (log.isDebugEnabled())
					log.debug("no no no 1! else if (maskData[jCenter*imgWidth + iCenter] & LINE_PATH != 0): i = " + j + ", maskData[stride + iCenter] = " + maskData[stride + iCenter] + ", iCenter = " + iCenter + ", jCenter = " + jCenter);
			}
		}
		lineSeg.xMidMean = (1.0*sumXMid)/(1.0*(yEnd - yBegin + 1));
		lineSeg.xMidMean2 = meanWXTot/(1.0*(yEnd - yBegin + 1));;

		/*PRINTIMGS
		log.debug("lineSeg.xMidMean: " + lineSeg.xMidMean + ", lineSeg.xMidMean2: " + lineSeg.xMidMean2 + ", lineSeg.xModel: " + lineSeg.xModel);
		log.debug("xMid1: " + xMid1 + ", xMid2: " + xMid1);
		log.debug("xBegin: " + xBegin + ", xEnd: " + xEnd);
		//*/

		return lineSeg;
	}
}


class LineSegPairs implements Comparable
{
	public LineSegHolder lineSeg1 = null;
	public LineSegHolder lineSeg2 = null;

	
	public LineSegPairs(LineSegHolder lineSeg1, LineSegHolder lineSeg2)
	{
		this.lineSeg1 = lineSeg1;
		this.lineSeg2 = lineSeg2;
		this.lineSeg1.setPairs.add(this);
		this.lineSeg2.setPairs.add(this);
	}


	public int compareTo(Object o)
	{
		LineSegPairs pair = (LineSegPairs) o;

		return (pair.lineSeg1.lineSeg.sizeSeg + pair.lineSeg2.lineSeg.sizeSeg) - 
				(lineSeg1.lineSeg.sizeSeg + lineSeg2.lineSeg.sizeSeg);
	}


	public String toString()
	{
		return "pair: " + lineSeg1.toString() + " -|- " + lineSeg2.toString();
	}
}
