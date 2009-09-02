package br.com.r4j.research.vline;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import java.text.*;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;



/**
 *
 * 
 * Armazena info de linhas 
 *
 *
 */
public class VLineRef  // extends VLine
{
	private static Log log = LogFactory.getLog(VLineRef.class.getName());

	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	public int band = -1;
	public int rLeft = -1;
	public int rRight = -1;
	public int gLeft = -1;
	public int gRight = -1;
	public int bLeft = -1;
	public int bRight = -1;

	private double x;
	private double y;

	private double thNornInv;

	private VLine lineAssociated = null;


	public VLineRef(double x, double y, int band, double thNornInv)
	{
		this.band = band;
		this.x = x;
		this.y = y;
		this.thNornInv = thNornInv;
	}

	
	public VLineRef(double x, double y, int band, double thNornInv, int rLeft, int rRight, int gLeft, int gRight, int bLeft, int bRight)
	{
		this.x = x;
		this.y = y;
		this.thNornInv = thNornInv;
		this.band = band;
		this.rLeft = rLeft;
		this.rRight = rRight;
		this.gLeft = gLeft;
		this.gRight = gRight;
		this.bLeft = bLeft;
		this.bRight = bRight;
	}


	public double getX()	{return this.x;}
	public void setX(double value)	{this.x = value;}

	public double getY()	{return this.y;}
	public void setY(double value)	{this.y = value;}

	public double getNormalInv()	{return this.thNornInv;}
	public void setNormalInv(double value)	{this.thNornInv = value;}


	public boolean isAssociated()
	{
		return lineAssociated != null;
	}


	public void setAssociatedLine(VLine line)
	{
		this.lineAssociated = line;
	}


	public VLine getAssociatedLine()
	{
		return this.lineAssociated;
	}


	public AbstractDoubleVector convert2vector()
	{
		AbstractDoubleVector vect = new DoubleVector(2);
		vect.setComponent(0, x);
		vect.setComponent(1, y);
		return vect;
	}


	public String toString()
	{
		return "ref: " + " -> x: " + numFrmt.format(x) + ",y: " + numFrmt.format(y);
	}
}

