
package br.com.r4j.research.vline;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;



/**
 *
 * Class que representa uma Linha vertical no espaço 3-D.
 * Armazena também o erro associado a localização da mesma.
 *
 * O erro é indicado por uma distribuição normal por enquanto.
 *
 * As coordenadas consideradas tem o eixo z paralelo a vertical.
 * 
 * A reta vertical é representada pelos valores em X e Y da mesma.
 *
 */
public class VLine
{
	private static Log log = LogFactory.getLog(VLine.class.getName());


	private double rho;
	private double beta;
	private double sinBeta;
	private double cosBeta;
	private double f;

	public int mapIdx = -1;
	public int mapPrevStateVectIdx = -1;
	public int mapCurrStateVectIdx = -1;

	/**
	 * O desvio-padrão dos erros.
	 */
	private AbstractDoubleSquareMatrix covar = null;


	public VLine(double x, double u, double f)
	{
		this.rho = x;
		this.beta = u;
		this.sinBeta = Math.sin(beta);
		this.cosBeta = Math.cos(beta);
		this.f = f;
	}

/*
	public VLine(AbstractDoubleVector vect, AbstractDoubleSquareMatrix covar)
	{
		this.x = vect.getComponent(0);
		this.y = vect.getComponent(1);
		this.covar = covar;
	}
//*/

	public double getRho()	{return this.rho;}
	public void setRho(double value)	{this.rho = value;}

//*
	public double getU()	{return this.f*Math.tan(beta);}
//	public void setBeta(double value)	{this.u = value*this.f/this.x;}
//*/
	public double getFocus()	{return this.f;}


	public double getBeta()	{return this.beta;}
	public double getCosBeta()	{return this.cosBeta;}
	public double getSinBeta()	{return this.sinBeta;}
	public void setBeta(double value)	
	{
		this.beta = value;
		this.sinBeta = Math.sin(beta);
		this.cosBeta = Math.cos(beta);
	}


	public AbstractDoubleSquareMatrix getCovar()	{return this.covar;}
	public void setCovar(AbstractDoubleSquareMatrix value)	{this.covar = value;}


	public AbstractDoubleVector convert2vector()
	{
		AbstractDoubleVector vect = new DoubleVector(2);
		vect.setComponent(0, rho);
		vect.setComponent(1, beta);
		return vect;
	}


	public String toString()
	{
		return "mapIdx: " + mapIdx + ",rho: " + getRho() + ", beta: " + getBeta();
	}
}

