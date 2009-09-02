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
public class VLineXY extends VLine
{
	private static Log log = LogFactory.getLog(VLineXY.class.getName());


	private double x;
	private double y;
	private double f;

	public int mapIdx = -1;
	public int mapPrevStateVectIdx = -1;
	public int mapCurrStateVectIdx = -1;

	/**
	 * O desvio-padrão dos erros.
	 */
	private AbstractDoubleSquareMatrix covar = null;


	public VLineXY(double x, double y, double f)
	{
		super(0, 0, f);
		this.x = x;
		this.y = y;
		this.f = f;
	}

/*
	public VLineXY(AbstractDoubleVector vect, AbstractDoubleSquareMatrix covar)
	{
		this.x = vect.getComponent(0);
		this.y = vect.getComponent(1);
		this.covar = covar;
	}
//*/

	public double getX()	{return this.x;}
	public void setX(double value)	{this.x = value;}

	public double getU()	{return this.y*this.f/this.x;}
//	public void setU(double value)	{this.y = value*this.f/this.x;}

	public double getY()	{return this.y;}
	public void setY(double value)	{this.y = value;}

	public AbstractDoubleSquareMatrix getCovar()	{return this.covar;}
	public void setCovar(AbstractDoubleSquareMatrix value)	{this.covar = value;}

	public AbstractDoubleVector convert2vector()
	{
		AbstractDoubleVector vect = new DoubleVector(2);
		vect.setComponent(0, x);
		vect.setComponent(1, y);
		return vect;
	}



	public String toString()
	{
		return "mapIdx: " + mapIdx + ",x: " + getX() + ", y: " + getY();
	}
}

