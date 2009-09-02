package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.*;


public class LineProjMatchPairSegment
{
	private LineProjMatchPair pair1 = null;
	private LineProjMatchPair pair2 = null;

	public LineProjMatchPairSegment next = null;

	public LineProjMatchPairSegment(LineProjMatchPair pair1, LineProjMatchPair pair2)
	{
		this.pair1 = pair1;
		this.pair2 = pair2;
	}


	public LineProjMatchPair getPair1()	{return pair1;}
	public LineProjMatchPair getPair2()	{return pair2;}


	public double getDShift()
	{
		double dShiftRef = (pair1.getRefProj().getU() - pair2.getRefProj().getU());
		double dShiftMeas = (pair1.getMeasProj().getU() - pair2.getMeasProj().getU());
		return (dShiftMeas - dShiftRef)/dShiftRef;
	}


 	public String toString()
	{
		String strNode = "{p1: " + pair1 + ", p2: " + pair2 + "}";
		if (next != null)
			return strNode + " -> " + next;
		else
			return strNode;
	}
}
