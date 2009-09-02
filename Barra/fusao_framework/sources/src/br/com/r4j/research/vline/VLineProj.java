package br.com.r4j.research.vline;

import java.util.List;
import java.text.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/**
 *
 * Class que representa uma projeção de linha vertical do espaço 3-D
 * no espaço 2-D.
 *
 * Armazena também o erro associado a localização da mesma.
 *
 * O erro é indicado por uma distribuição normal por enquanto.
 *
 * As coordenadas consideradas do plano de projeção tem na vertical
 * o eixo z projetado no eixo v e na horizontal o eixo y projetado no
 * eixo u.
 * 
 * A reta vertical é representada pelo valor em u da mesma.
 *
 */
public class VLineProj implements Comparable
{
	private static Log log = LogFactory.getLog(VLineProj.class.getName());
	private static Log logPerfil = LogFactory.getLog("perfil_gen");

	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}


	public int mapIdx = -1;

	private LineSegmentCollection lCol = null;
/*
	private int [] arrayImg = null;
	private int imgWidth, 
	private int imgHeight
//*/

	/**
	 * O desvio-padrão dos erros.
	 */
	private double uErrorSigma;

	private VLinePerfil perfil = null;



	public VLineProj(LineSegmentCollection lCol) //, int [] arrayImg, int imgWidth, , int imgHeight)
	{
		this.lCol = lCol;
		this.uErrorSigma = 0;
	}


	public VLineProj(LineSegmentCollection lCol, double uErrorSigma)
	{
		this.lCol = lCol;
		this.uErrorSigma = uErrorSigma;
	}


	public VLineProj(double u, double uErrorSigma)
	{
		LineSegment lineSeg = new LineSegment();
		lineSeg.xModel = u;
		lineSeg.xIni = -99999;
		lineSeg.yIni = -99999;
		lineSeg.xEnd = 99999;
		lineSeg.yEnd = 99999;
		lineSeg.sizeSeg = 99999*2;
		lineSeg.xMidle = u;

		this.lCol = new LineSegmentCollection(lineSeg);
		this.uErrorSigma = uErrorSigma;
	}


	public double getU()	{return this.lCol.xMidMean2;}
//	public double getU()	{return this.lCol.getXMidleOfSegment();}

	public double getUErrorSigma()	{return this.uErrorSigma;}
	public void setUErrorSigma(double value)	{this.uErrorSigma = value;}

	public int getVBegin()	{return this.lCol.yIni;}
	public int getVEnd()	{return this.lCol.yEnd;}

	public VLinePerfil getPerfil(){return this.perfil;}


	public void calculatePerfil(int [] arrayImg, int imgWidth, int imgHeight)
	{
		perfil = new VLinePerfil();


		List listSegs = lCol.listSegs;
//		log.debug("calculatePerfil: lCol.listSegs.size(): " + lCol.listSegs.size());
		for (int i = 0; i < listSegs.size(); i++)
		{
			LineSegment ls = (LineSegment) listSegs.get(i);
			perfil.add(arrayImg, imgWidth, imgHeight, ls);
		}

		logPerfil.debug("u: " + getU());
		perfil.debugPerfil(logPerfil, "\t");
	}


	public LineSegmentCollection getSegment()
	{
		return lCol;
	}


	public int compareTo(Object o)
	{
		return (int) ((lCol.xMidle - ((VLineProj) o).getU())*10);
	}

 
 	public String toString()
	{
		return "proj(" + numFrmt.format(getU()) + ")";
	}
}

