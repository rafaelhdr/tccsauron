package br.com.r4j.research.image.sequence.featurematch.vline;

import java.text.*;

import br.com.r4j.research.vline.*;
import br.com.r4j.graph.*;


public class LineProjMatchPair
{
	private VLineRefProj lRefProj = null;
	private VLineProj lMeasProj = null;
	private double comp = 0;
	private INode node = null;

	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}


	public LineProjMatchPair(VLineRefProj lRefProj, VLineProj lMeasProj, double comp)
	{
		this.lRefProj = lRefProj;
		this.lMeasProj = lMeasProj;
		this.comp = comp;
	}


	public double getComp()	{return comp;}
	public VLineRefProj getRefProj()	{return lRefProj;}
	public VLineProj getMeasProj()	{return lMeasProj;}

	public INode getNode()	{return node;}
	public void setNode(INode nd)	{this.node = nd;}


	public double getUShift()
	{
		return lRefProj.getU() - lMeasProj.getU();
	}


 	public String toString()
	{
		return "[map: " + lRefProj + ", emas: " + lMeasProj + ", comp: " + numFrmt.format(comp) + ", u shift:" + numFrmt.format(getUShift()) + "]";
	}
}
