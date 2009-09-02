package br.com.r4j.research.vline;

import java.util.List;
import java.text.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/**
 *
 * Class que representa uma proje��o de linha vertical do espa�o 3-D
 * no espa�o 2-D.
 *
 * Armazena tamb�m o erro associado a localiza��o da mesma.
 *
 * O erro � indicado por uma distribui��o normal por enquanto.
 *
 * As coordenadas consideradas do plano de proje��o tem na vertical
 * o eixo z projetado no eixo v e na horizontal o eixo y projetado no
 * eixo u.
 * 
 * A reta vertical � representada pelo valor em u da mesma.
 *
 */
public class VLineRefProj implements Comparable
{
	private static Log log = LogFactory.getLog(VLineRefProj.class.getName());
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private VLineRef lRef = null;
	private double u = -1;


	/**
	 * O desvio-padr�o dos erros.
	 */
	private double uErrorSigma;

	private VLinePerfil perfil = null;



	public VLineRefProj(double u, VLineRef lRef) //, int [] arrayImg, int imgWidth, , int imgHeight)
	{
		this.u = u;
		this.lRef = lRef;
	}

	public double getU()	{return this.u;}


	public VLineRef getLineRef()
	{
		return lRef;
	}


	public int compareTo(Object o)
	{
		return (int) ((u - ((VLineRefProj) o).getU())*10);
	}

 
 	public String toString()
	{
		return "refproj(" + numFrmt.format(getU()) + ")";
	}
}

