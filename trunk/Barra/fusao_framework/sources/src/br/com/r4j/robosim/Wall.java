package br.com.r4j.robosim;

import java.io.*;
import java.rmi.*;
import java.awt.Point;
import java.util.*;

import java.awt.geom.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.*;

import cern.colt.matrix.*;
import cern.colt.matrix.impl.*;
import cern.colt.matrix.linalg.*;

import br.com.r4j.math.geom.*;
//import br.com.r4j.jmr.*;


/**
 */
public class Wall extends Line2D
{
	private static Log log = LogFactory.getLog(Wall.class.getName());

	private double x1 = 0;
	private double y1 = 0;
	private double x2 = 0;
	private double y2 = 0;
	private boolean b1IsCorner = false;
	private boolean b2IsCorner = false;
	private Wall previous = null;
	private Wall next = null;

	private double pR = 0;
	private double pTheta = 0;
	private double pThetaCos = 0;
	private double pThetaSin = 0;

	private ArrayList listVLines = null;

	private double xMass = 0;
	private double yMass = 0;

	public Wall()
	{
		listVLines = new ArrayList();
	}

	public Point2D getP1()	{return new Point2D.Double(x1, y1);}
	public Point2D getP2()	{return new Point2D.Double(x2, y2);}

	public double getX1()	{return x1;}
	public void setX1(double a)	{x1 = a; this.calculate();}

	public double getX2()	{return x2;}
	public void setX2(double a)	{x2 = a; this.calculate();}

	public double getY1()	{return y1;}
	public void setY1(double a)	{y1 = a; this.calculate();}

	public double getY2()	{return y2;}
	public void setY2(double a)	{y2 = a; this.calculate();}

	public void setLine(double X1, double Y1, double X2, double Y2)
	{
		this.x1 = X1;
		this.y1 = Y1;
		this.x2 = X2;
		this.y2 = Y2;

		this.calculate();
	}

	public Rectangle2D getBounds2D()	{return super.getBounds();}

	public boolean is1Corner()	{return b1IsCorner;}
	public void setIs1Corner(boolean a)	{b1IsCorner = a;}

	public boolean is2Corner()	{return b2IsCorner;}
	public void setIs2Corner(boolean a)	{b2IsCorner = a;}

	public double getD()	{return pR;}
	public double getTheta()	{return pTheta;}
	public double getCosTheta()	{return pThetaCos;}
	public double getSinTheta()	{return pThetaSin;}

	public double getXMass()	{return xMass;}
	public double getYMass()	{return yMass;}


	public void addVerticalLine(Point2D ptVLine) {listVLines.add(ptVLine);}
	public List getVerticalLines() {return listVLines;}


	public Wall previous()	{return previous;}
	public Wall next()	{return next;}

	public void setPrevious(Wall a)	{previous = a;}
	public void setNext(Wall a)	{next = a;}

	public void calculate()
	{
		if (Math.abs(x2 - x1) < 0.0001)
		{
			pR = Math.abs(x2 + x1)/2;
			if (x2 > 0)
			{
				pTheta = 0;
				pThetaCos = 1;
				pThetaSin = 0;
			}
			else
			{
				pTheta = Math.PI;
				pThetaCos = -1;
				pThetaSin = 0;
			}
		}
		else if (Math.abs(y2 - y1) < 0.0001)
		{
			pR = Math.abs(y2 + y1)/2;
			if (y2 > 0)
			{
				pTheta = Math.PI/2;
				pThetaCos = 0;
				pThetaSin = 1;
			}
			else
			{
				pTheta = 3*Math.PI/2;
				pThetaCos = 0;
				pThetaSin = -1;
			}
		}
		else
		{
			double m1 = (y2 - y1) / (x2 - x1);
			double m2 = -1 / m1;

			double xDist = (y1 - m1*x1) / (m2 - m1);
			double yDist = m2*xDist;

			pR = Math.sqrt(xDist*xDist + yDist*yDist);
			pTheta = Math.atan(m2);
			if (pTheta < 0)
				pTheta += Math.PI*2;
			pThetaCos = Math.cos(pTheta);
			pThetaSin = Math.sin(pTheta);
		}

		xMass = (x1 + x2)/2;
		yMass = (y1 + y2)/2;
	}


	public String toString()
	{
		return "(" + pR + ":" + pTheta + ") (" + x1 + ":" + y1 + "," + x2 + ":" + y2 + ")";
	}
}


