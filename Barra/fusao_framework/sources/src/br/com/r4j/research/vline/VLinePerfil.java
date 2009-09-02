package br.com.r4j.research.vline;

import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.Arrays;
import br.com.r4j.image.operation.oneband.colorspace.HistogramEqualizationOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.math.ArrayMath;
import br.com.r4j.image.operation.threebandpacked.*;


/**
 *
 *
 */
public class VLinePerfil
{
	private static Log log = LogFactory.getLog(VLinePerfil.class.getName());
	private static Log logComp = LogFactory.getLog("comparisons");

	public static int PERFIL_SIZE = 10;
	public static int PERFIL_HISTORICO = 3;
	public static double MAX_ATTENUATION = 0.95;
	
	public static boolean APPLY_HISTOGRAM = true;

	private int [] perfilRHist = null;
	private int [] perfilGHist = null;
	private int [] perfilBHist = null;

	private int [] perfilR = null;
	private int [] perfilG = null;
	private int [] perfilB = null;

	
	private int perfilSize = -1;


	// Número de perfís já usados.
	private int perfisUsadosLength = 0;
	private int perfisUsadosHistorico = 0;

	private static double [] weightAttenuation = null;
	private static double weightNorm;
	private static ArrayList listHistOps = null;
	private static boolean initialized = false;
	
	// Limiar para perfis de marcos.
	private int I_lim = 30;
	public static int LEFT_BIGGER = -2;
	public static int RIGHT_BIGGER = -3;
	public static int NODIFF = -1;

	
	public static void initialize()
	{
		if (!initialized)
		{
			listHistOps = new ArrayList();
			HistogramEqualizationOneInputImageOp histOp = new HistogramEqualizationOneInputImageOp();
			listHistOps.add(histOp);

			int sideBandSize = PERFIL_SIZE / 2;
			double variance = - sideBandSize*sideBandSize/4/Math.log(MAX_ATTENUATION);
			weightAttenuation = new double[PERFIL_SIZE];
			weightNorm = 0;
			for (int i = 0; i < PERFIL_SIZE; i++)
			{
				double x = i - sideBandSize;
				weightAttenuation[i] = Math.exp(-1*x*x/4/variance);
				weightNorm += weightAttenuation[i];
			}
			weightNorm = 1.0 * PERFIL_SIZE / weightNorm;
			log.debug("attenuation vector: " + br.com.r4j.commons.util.Arrays.toString(weightAttenuation, weightAttenuation.length, 2, 4));
			log.debug("attenuation nrom: " + weightNorm);
			initialized = true;
		}
	}


	public VLinePerfil()
	{
		initialize();
		perfisUsadosHistorico = 1;
		perfisUsadosLength = 0;
	}


	protected static HistogramEqualizationOneInputImageOp getHistOp()
	{
		if (listHistOps.size() > 0)
			return (HistogramEqualizationOneInputImageOp) listHistOps.remove(listHistOps.size() - 1);
		else 
			return new HistogramEqualizationOneInputImageOp();
	}


	protected static void returnHistOp(HistogramEqualizationOneInputImageOp histOp)
	{
		listHistOps.add(histOp);
	}


	public boolean isPerfilAvailable()
	{
		return perfilG != null;
	}


	public int getPerfilVSize()
	{
		return perfisUsadosLength;
	}


	/**
	 * Adiciona um novo perfil
	 *
	 *
	 * @param vLineUp limite superior até onde a linha chega
	 * @param vLineDown limite inferior até onde a linha chega
	 */
	public void add(int [] arrayImg, int imgWidth, int imgHeight, LineSegment ls)
	{
		int [] perfilNovoR = new int [PERFIL_SIZE], perfilNovoG = new int [PERFIL_SIZE], perfilNovoB = new int [PERFIL_SIZE], perfilNovoIllu = new int [PERFIL_SIZE], perfilNovoIlluEq = new int [PERFIL_SIZE];
/*
		Arrays.fill(perfilNovoR, 0, perfilNovoR.length, 0);
		Arrays.fill(perfilNovoG, 0, perfilNovoG.length, 0);
		Arrays.fill(perfilNovoB, 0, perfilNovoB.length, 0);
//		Arrays.fill(perfilNovoIllu, 0, perfilNovoIllu.length, 0);
//*/

		// Soma todos os valores do perfil numa única linha
		for (int v = ls.yIni; v <= ls.yEnd; v++)
		{
			int lineIdx = v*imgWidth;
			int uBegin = (int) (ls.mModel*v + ls.xModel - PERFIL_SIZE/2), uEnd = (int) uBegin + PERFIL_SIZE;
			if (uBegin < 0) uBegin = 0;
			if (uEnd >= imgWidth) uEnd = imgWidth - 1;
			for (int u = uBegin; u < uBegin + PERFIL_SIZE; u++)
			{
				int pix = arrayImg[lineIdx + u];
				int perfilIdx = u - uBegin;
				perfilNovoR[perfilIdx] += ThreeBandPackedUtil.getRed(pix);
				perfilNovoG[perfilIdx] += ThreeBandPackedUtil.getGreen(pix);
				perfilNovoB[perfilIdx] += ThreeBandPackedUtil.getBlue(pix);
			}
		}
		int perfilSizeV = ls.yEnd - ls.yIni + 1;
		for (int u = 0; u < PERFIL_SIZE; u++)
		{
			perfilNovoR[u] /= perfilSizeV;
			perfilNovoG[u] /= perfilSizeV;
			perfilNovoB[u] /= perfilSizeV;
			perfilNovoIllu[u] = (int) (0.3f * (float) perfilNovoR[u] + 0.59f * (float) perfilNovoG[u] + 0.11f * (float) perfilNovoB[u]);
//			perfilNovoIllu[u] = (perfilNovoR[u] + perfilNovoG[u] + perfilNovoB[u])/3;
		}
		perfilSize = perfilSizeV;

		// Equaliza o novo pedaço do perfil.
/*
		HistogramEqualizationOneInputImageOp histOp = VLinePerfil.getHistOp();
		histOp.setRowStart(-1);
		histOp.operate(perfilNovoIllu, perfilNovoIlluEq, PERFIL_SIZE, 1);
/*
		histOp.operate(perfilNovoR, perfilNovoR, PERFIL_SIZE, 1);
		histOp.operate(perfilNovoG, perfilNovoG, PERFIL_SIZE, 1);
		histOp.operate(perfilNovoB, perfilNovoB, PERFIL_SIZE, 1);
		VLinePerfil.returnHistOp(histOp);
//*/
		if (APPLY_HISTOGRAM) for (int u = 0; u < PERFIL_SIZE; u++)
		{
//			float factor = 1.0f * perfilNovoIllu[u] / (1.0f * perfilNovoIllu[u]);
			float factor = 1.0f;
			perfilNovoR[u] = (int) (1.0f * perfilNovoR[u] * factor);
			perfilNovoG[u] = (int) (1.0f * perfilNovoG[u] * factor);
			perfilNovoB[u] = (int) (1.0f * perfilNovoB[u] * factor);
		}


		if (perfilR == null)
		{
			perfilR = perfilNovoR; perfilG = perfilNovoG; perfilB = perfilNovoB;
/*
			Arrays.arrayCopy(perfilNovoR, perfilR);
			Arrays.arrayCopy(perfilNovoG, perfilG);
			Arrays.arrayCopy(perfilNovoB, perfilB);
//*/
		}
		else
		{
			for (int u = 0; u < PERFIL_SIZE; u++)
			{
				perfilR[u] = (perfisUsadosLength*perfilR[u] + ls.sizeSeg*perfilNovoR[u])/(perfisUsadosLength + ls.sizeSeg);
				perfilG[u] = (perfisUsadosLength*perfilG[u] + ls.sizeSeg*perfilNovoG[u])/(perfisUsadosLength + ls.sizeSeg);
				perfilB[u] = (perfisUsadosLength*perfilB[u] + ls.sizeSeg*perfilNovoB[u])/(perfisUsadosLength + ls.sizeSeg);
			}
		}
		perfisUsadosLength += ls.sizeSeg;

		if (log.isDebugEnabled())
		{
			log.debug("PERFIL:Tamanho: " + perfilR.length);
			log.debug("perfilR: " + Arrays.toString(perfilR, perfilR.length, 3));
			log.debug("perfilG: " + Arrays.toString(perfilG, perfilG.length, 3));
			log.debug("perfilB: " + Arrays.toString(perfilB, perfilB.length, 3));
		}
	}


	
	/** 
	 * Adiciona um perfil pré-calculado. Considera que ele é apenas uma medida a mais.
	 */
//*
	public void addHistoric(VLinePerfil linePerfilComp)
	{
		int [] perfilNovoR = linePerfilComp.getPerfilRed(), perfilNovoG = linePerfilComp.getPerfilGreen(), perfilNovoB = linePerfilComp.getPerfilBlue();

		if (perfisUsadosHistorico < PERFIL_HISTORICO)
			perfisUsadosHistorico++;

		for (int u = 0; u < PERFIL_SIZE; u++)
		{
			perfilR[u] = ((perfisUsadosHistorico - 1)*perfilR[u] + perfilNovoR[u])/perfisUsadosHistorico;
			perfilG[u] = ((perfisUsadosHistorico - 1)*perfilG[u] + perfilNovoG[u])/perfisUsadosHistorico;
			perfilB[u] = ((perfisUsadosHistorico - 1)*perfilB[u] + perfilNovoB[u])/perfisUsadosHistorico;
		}
	}

//*/

	/**
	 * Compara o perfil armazenado com um outro perfil e retorna o quão parecido ambos são.
	 * Quanto maior o valor, masi parecidos são.
	 */
	public double compareRed(VLinePerfil linePerfilComp) {
		return this.doComparation(perfilR, linePerfilComp.getPerfilRed(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilR.length);}
	public double compareRedFirstHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilR, linePerfilComp.getPerfilRed(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilR.length/2 + 1);}
	public double compareRedSecondHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilR, linePerfilComp.getPerfilRed(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),perfilR.length/2, perfilR.length/2 + 1);}


	/**
	 * Compara o perfil armazenado com um outro perfil e retorna o quão parecido ambos são.
	 * Quanto maior o valor, masi parecidos são.
	 */
	public double compareGreen(VLinePerfil linePerfilComp) {
		return this.doComparation(perfilG, linePerfilComp.getPerfilGreen(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilG.length);}
	public double compareGreenFirstHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilG, linePerfilComp.getPerfilGreen(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilG.length/2 + 1);}
	public double compareGreenSecondHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilG, linePerfilComp.getPerfilGreen(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),perfilG.length/2, perfilG.length/2 + 1);}


	/**
	 * Compara o perfil armazenado com um outro perfil e retorna o quão parecido ambos são.
	 * Quanto maior o valor, masi parecidos são.
	 */
	public double compareBlue(VLinePerfil linePerfilComp) {
		return this.doComparation(perfilB, linePerfilComp.getPerfilBlue(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilB.length);}
	public double compareBlueFirstHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilB, linePerfilComp.getPerfilBlue(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),0, perfilB.length/2 + 1);}
	public double compareBlueSecondHalf(VLinePerfil linePerfilComp){
		return this.doComparation(perfilB, linePerfilComp.getPerfilBlue(), perfisUsadosLength, linePerfilComp.getPerfilVSize(),perfilB.length/2, perfilB.length/2 + 1);}


	protected double doComparation(int [] perfil_1, int [] perfil_2, int perfilVSize_1, int perfilVSize_2, int offset, int length)
	{
//		return Math.sqrt(ArrayMath.correlation(perfil_1, perfil_2))*100;

/*
		return cov;
//*/

		double mean1 = ArrayMath.mean(perfil_1, offset, length);
		double mean2 = ArrayMath.mean(perfil_2, offset, length);
		double cov = ArrayMath.metricCorrPercentage(perfil_1, perfil_2, 
													offset, length, 
													offset, length, 
													mean1, mean2, weightAttenuation);

		double sub = ArrayMath.subAllAbs(perfil_1, perfil_2, 
													offset, length, 
													offset, length);
		double diffMult = 1 - sub*1.0/((length - offset)*150.0);
		double diffSize = 1 - Math.abs(1.0*perfilVSize_2 - 1.0*perfilVSize_1)/(150.0);
		if (diffMult < 0)
			diffMult = 0;
		if (diffSize < 0.5)
			diffSize = 0.5;

		logComp.debug("doComparation:cov: " + cov + ", sub: " + sub + ", diffMult: " + diffMult + ", (perfilVSize_2 - perfilVSize_1): " + (perfilVSize_2 - perfilVSize_1) + ", diffSize: " + diffSize + ", result: " + cov*diffMult*diffSize*1.1);
 
		return cov*diffMult*diffSize*1.1;

/*
		double cov1minus = ArrayMath.metricCorrPercentage(perfil_1, perfil_2, 1, perfil_1.length - 1, 0, perfil_2.length - 1, mean1, mean2);
		if (cov < cov1minus)
			cov = cov1minus;
		double cov1plus = ArrayMath.metricCorrPercentage(perfil_1, perfil_2, 0, perfil_1.length - 1, 1, perfil_2.length - 1, mean1, mean2);
		if (cov < cov1plus)
			cov = cov1plus;
//*/
	}


	/**
	 * Compara o perfil armazenado com um outro perfil e retorna o quão parecido ambos são.
	 * Quanto maior o valor, masi parecidos são.
	 */
	public double compare(VLinePerfil linePerfilComp)
	{
		logComp.debug("----------- compare");
		double red = this.compareRed(linePerfilComp);
		double green = this.compareGreen(linePerfilComp);
		double blue = this.compareBlue(linePerfilComp);

		logComp.debug("perfilR: " + Arrays.toString(perfilR, perfilR.length, 3));
		logComp.debug("perfilR: " + Arrays.toString(linePerfilComp.getPerfilRed(), perfilR.length, 3));
		logComp.debug("---red- " + red);

		logComp.debug("perfilG: " + Arrays.toString(perfilG, perfilG.length, 3));
		logComp.debug("perfilG: " + Arrays.toString(linePerfilComp.getPerfilGreen(), perfilG.length, 3));
		logComp.debug("-green- " + green);

		logComp.debug("perfilB: " + Arrays.toString(perfilB, perfilB.length, 3));
		logComp.debug("perfilB: " + Arrays.toString(linePerfilComp.getPerfilBlue(), perfilB.length, 3));
		logComp.debug("--blue- " + blue);

		return red < blue ? (red < green ? red : green) : (blue < green ? blue : green);
	}


	public void debugPerfil(Log logD, String append)
	{
		double mean1r = ArrayMath.mean(perfilR, 0, perfilR.length/2);
		double mean1g = ArrayMath.mean(perfilG, 0, perfilG.length/2);
		double mean1b = ArrayMath.mean(perfilB, 0, perfilB.length/2);
		double mean2r = ArrayMath.mean(perfilR, perfilR.length/2, perfilR.length/2);
		double mean2g = ArrayMath.mean(perfilG, perfilG.length/2, perfilG.length/2);
		double mean2b = ArrayMath.mean(perfilB, perfilB.length/2, perfilB.length/2);
		int rSide = mean1r > mean2r + I_lim ? LEFT_BIGGER : mean2r > mean1r + I_lim ? RIGHT_BIGGER : NODIFF;
		int gSide = mean1g > mean2g + I_lim ? LEFT_BIGGER : mean2g > mean1g + I_lim ? RIGHT_BIGGER : NODIFF;
		int bSide = mean1b > mean2b + I_lim ? LEFT_BIGGER : mean2b > mean1b + I_lim ? RIGHT_BIGGER : NODIFF;
		logD.debug(append + "mean1: " + mean1r + ", " + mean1g + ", " + mean1b + "(RGB)");
		logD.debug(append + "mean2: " + mean2r + ", " + mean2g + ", " + mean2b + "(RGB)");
		logD.debug(append + "rSide:    " + rSide +    ", gSide:    " + gSide +    ", bSide:    " + bSide);
		logD.debug(append + "perfilR: " + Arrays.toString(perfilR, perfilR.length, 3));
		logD.debug(append + "perfilG: " + Arrays.toString(perfilG, perfilG.length, 3));
		logD.debug(append + "perfilB: " + Arrays.toString(perfilB, perfilB.length, 3));
	}


	public double getMeanR1()	{return ArrayMath.mean(perfilR, 0, perfilR.length/2);}
	public double getMeanG1()	{return ArrayMath.mean(perfilG, 0, perfilG.length/2);}
	public double getMeanB1()	{return ArrayMath.mean(perfilB, 0, perfilB.length/2);}
	public double getMeanR2()	{return ArrayMath.mean(perfilR, perfilR.length/2, perfilR.length/2);}
	public double getMeanG2()	{return ArrayMath.mean(perfilG, perfilG.length/2, perfilG.length/2);}
	public double getMeanB2()	{return ArrayMath.mean(perfilB, perfilB.length/2, perfilB.length/2);}


	public double compare(VLineRef lRef)
	{
		log.debug("----------- compare");
		double mean1r = ArrayMath.mean(perfilR, 0, perfilR.length/2);
		double mean1g = ArrayMath.mean(perfilG, 0, perfilG.length/2);
		double mean1b = ArrayMath.mean(perfilB, 0, perfilB.length/2);
		double mean2r = ArrayMath.mean(perfilR, perfilR.length/2, perfilR.length/2);
		double mean2g = ArrayMath.mean(perfilG, perfilG.length/2, perfilG.length/2);
		double mean2b = ArrayMath.mean(perfilB, perfilB.length/2, perfilB.length/2);

		log.debug("mean1: " + mean1r + ", " + mean1g + ", " + mean1b);
		log.debug("mean2: " + mean2r + ", " + mean2g + ", " + mean2b);
		log.debug("lRef.Left: " + lRef.rLeft + ", " + lRef.gLeft + ", " + lRef.bLeft);
		log.debug("lRef.Right: " + lRef.rRight + ", " + lRef.gRight + ", " + lRef.bRight);
		
		// Se é comparaçào 'binária'.
		if ((Math.abs(lRef.rRight - lRef.rLeft) == 255 || Math.abs(lRef.rRight - lRef.rLeft) == 0))
		{
			int rSide = mean1r > mean2r + I_lim ? LEFT_BIGGER : mean2r > mean1r + I_lim ? RIGHT_BIGGER : NODIFF;
			int gSide = mean1g > mean2g + I_lim ? LEFT_BIGGER : mean2g > mean1g + I_lim ? RIGHT_BIGGER : NODIFF;
			int bSide = mean1b > mean2b + I_lim ? LEFT_BIGGER : mean2b > mean1b + I_lim ? RIGHT_BIGGER : NODIFF;
			
			int rSideRef = lRef.rLeft > lRef.rRight + I_lim ? LEFT_BIGGER : lRef.rRight > lRef.rLeft + I_lim ? RIGHT_BIGGER : NODIFF;
			int gSideRef = lRef.gLeft > lRef.gRight + I_lim ? LEFT_BIGGER : lRef.gRight > lRef.gLeft + I_lim ? RIGHT_BIGGER : NODIFF;
			int bSideRef = lRef.bLeft > lRef.bRight + I_lim ? LEFT_BIGGER : lRef.bRight > lRef.bLeft + I_lim ? RIGHT_BIGGER : NODIFF;
			log.debug("rSide:    " + rSide +    ", gSide:    " + gSide +    ", bSide:    " + bSide);
			log.debug("rSideRef: " + rSideRef + ", gSideRef: " + gSideRef + ", bSideRef: " + bSideRef);

			return (rSideRef == rSide && gSideRef == gSide && gSideRef == gSide) ? 1 : 0;
		}
		else
		{
			double diffMean_r_proj = mean1r - mean2r;
			double diffMean_g_proj = mean1g - mean2g;
			double diffMean_b_proj = mean1b - mean2b;
			double diffMean_r_ref = lRef.rLeft - lRef.rRight;
			double diffMean_g_ref = lRef.gLeft - lRef.gRight;
			double diffMean_b_ref = lRef.bLeft - lRef.bRight;

			log.debug("diffMean_r_proj: " + diffMean_r_proj + ", diffMean_g_proj: " + diffMean_g_proj + ", diffMean_b_proj: " + diffMean_b_proj);
			log.debug("diffMean_r_ref:  " + diffMean_r_ref +  ", diffMean_g_ref:  " + diffMean_g_ref +  ", diffMean_b_ref:  " + diffMean_b_ref);

			boolean diffMean_r_projPositive = diffMean_r_proj > 0;
			boolean diffMean_g_projPositive = diffMean_g_proj > 0;
			boolean diffMean_b_projPositive = diffMean_b_proj > 0;
			boolean diffMean_r_refPositive = diffMean_r_ref > 0;
			boolean diffMean_g_refPositive = diffMean_g_ref > 0;
			boolean diffMean_b_refPositive = diffMean_b_ref > 0;

			if (diffMean_r_projPositive == diffMean_r_refPositive &&
				diffMean_g_projPositive == diffMean_g_refPositive &&
				diffMean_b_projPositive == diffMean_b_refPositive)
			{
				double diffDiff3 =	(Math.abs(diffMean_r_proj - diffMean_r_ref) +
								Math.abs(diffMean_g_proj - diffMean_g_ref) + 
								Math.abs(diffMean_b_proj - diffMean_b_ref))/3;
				log.debug("diffDiff3: " + diffDiff3);

				double meanProj = mean1r + mean2r + mean1g + mean2g + mean1b + mean2b;
				double meanRef = lRef.rLeft + lRef.rRight + lRef.gLeft + lRef.gRight + lRef.bLeft + lRef.bRight;
				double diffMean = Math.abs(meanProj - meanRef)/6;
				log.debug("diffMean: " + diffMean);

				if (diffDiff3 > 30 || diffMean > 100)
					return 0;
				else
				{
					log.debug("result: " + (255 + 2*120 - (2*diffDiff3 + diffMean)));
//					return 255 + 100 - diffDiff3 - diffMean*1.0;

					return (0.7*(30 - diffDiff3) + 0.3*(100 - diffMean))/(0.7*(30) + 0.3*(100));
				}
			}
			else
				return 0;
		}
	}


	public double compareFirstHalf(VLinePerfil linePerfilComp)
	{
		double red = this.compareRedFirstHalf(linePerfilComp);
		double blue = this.compareGreenFirstHalf(linePerfilComp);
		double green = this.compareBlueFirstHalf(linePerfilComp);
		return red < blue ? (red < green ? red : green) : (blue < green ? blue : green);

//		(this.compareRedFirstHalf(linePerfilComp) +  this.compareGreenFirstHalf(linePerfilComp) +  this.compareBlueFirstHalf(linePerfilComp))/3;
	}

	public double compareSecondHalf(VLinePerfil linePerfilComp)
	{
		double red = this.compareRedSecondHalf(linePerfilComp);
		double blue = this.compareGreenSecondHalf(linePerfilComp);
		double green = this.compareBlueSecondHalf(linePerfilComp);
		return red < blue ? (red < green ? red : green) : (blue < green ? blue : green);
//		(this.compareRedSecondHalf(linePerfilComp) + this.compareGreenSecondHalf(linePerfilComp) +  this.compareBlueSecondHalf(linePerfilComp))/3;
	}


	public int [] getPerfilRed()	{return perfilR;}
	public int [] getPerfilGreen()	{return perfilG;}
	public int [] getPerfilBlue()	{return perfilB;}
}

// 40995.19540 56300.000009 01140.806744 2 30940000021612

