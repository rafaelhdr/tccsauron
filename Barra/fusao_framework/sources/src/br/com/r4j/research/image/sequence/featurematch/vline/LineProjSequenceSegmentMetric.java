package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.*;
import java.util.List;
import java.text.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


public class LineProjSequenceSegmentMetric implements Comparable
{
	private static Log log = LogFactory.getLog(LineProjSequenceSegmentMetric.class.getName());

	public double uShift = 0;
	public double uVar = 0;
	public double sumComp = 0;
	public int countPairs = 0;
	public double countPairsSqrt = 0;
	public int countMax = 0;
	public double minU = 0; 
	public double maxU = 0;

	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}


	public LineProjSequenceSegmentMetric(double uShift, double uVar, double minU, double maxU, double sumComp, int countPairs, int countMax)
	{
		this.uShift = uShift;
		this.uVar = uVar;
		this.sumComp = sumComp;
		this.countPairs = countPairs;
		countPairsSqrt = Math.sqrt(countPairs);
		this.minU = minU;
		this.maxU = maxU;
	}


	public double getMetric()
	{
		double dDisp = (1.0*(Math.abs(maxU - minU)))/((320.0)) + 1.0;
//		double uShiftClosenessToReality = uShift*uShift + 70*70;
//		double uShiftClosenessToReality = Math.abs(uShift) + 20;
//		double perfilComp = (50 + sumComp*50/355/countPairs);
//		double perfilComp = (20 + sumComp);
		double perfilComp = (sumComp);
		double uVarComp = (3 + uVar + Math.abs(maxU - minU)/10.0 + Math.abs(uShift)/10.0);
//		double countPairsComp = countPairsSqrt;

//		double fShift = 1.0*countPairs/(uVarComp + Math.abs(maxU - minU)*Math.abs(uShift)/300.0);
		double fShift = 1.0*Math.sqrt(countPairs)/(uVarComp + Math.abs(maxU - minU)*Math.abs(uShift)/100.0);

		//*PRINTIMGS
		log.debug("Math.abs(maxU - minU): " + Math.abs(maxU - minU) + ", Math.abs(uShift): " + Math.abs(uShift) + ", sumComp: " + sumComp + ", uVarComp: " + uVarComp);
//		log.debug("perfilComp: " + perfilComp + ", countPairs: " + countPairs + ", dDisp: " + dDisp + ", uVarComp: " + uVarComp);
		log.debug("fShift: " + fShift + ", Math.abs(maxU - minU)*Math.abs(uShift)/100.0: " + Math.abs(maxU - minU)*Math.abs(uShift)/100.0);
		log.debug("sumComp: " + sumComp + ", sumComp + 7.5*fShift: " + (sumComp + 7.5*fShift));
		//*/
//		double metricVal = 100*perfilComp*countPairs*dDisp/(uVarComp + 0.0001);
		double metricVal = sumComp + 7.5*fShift;
//		double metricVal = sumComp + 10000*fShift;
		return metricVal;
	}


	public int compareTo(Object o)
	{
		LineProjSequenceSegmentMetric metr = (LineProjSequenceSegmentMetric) o;

//		double sizeComp = (countMax*1.1 - metr.countPairs)/(countMax*1.1 - countPairs);

/*
		double uVarComp = (3 + metr.uVar)/(uVar + 3);
		double uDstComp = (maxU - minU)/(metr.maxU - metr.minU);
		double perfilComp = (50 + sumComp*50/355/countPairs);
		double sizeComp = (countPairs*1.0)/(metr.countPairs*1.0 + 0.01);

		if (uVarComp > 0)
		{
			log.debug("comp:" + this + " || " + metr + ": " + numFrmt.format(uVarComp) + ", " + numFrmt.format(perfilComp) + ", " + numFrmt.format(sizeComp) + ", " + numFrmt.format(uDstComp));
		}
		else
		{
			uVarComp = -10000;
			log.debug("comp:" + this + " || " + metr + ": ???????, " + numFrmt.format(perfilComp) + ", " + numFrmt.format(sizeComp) + ", " + numFrmt.format(uDstComp));
		}
//*/
//		return (int) (10000*(1 - uVarComp*perfilComp*sizeComp*uDstComp));
//		return (int) (10000*(1 - uVarComp*sizeComp*uDstComp));
		return (int) (10000*(metr.getMetric() - getMetric()));
	}

 
 	public String toString()
	{
/*
		double dDisp = (1.0*(Math.abs(maxU - minU)))/((320.0)) + 1.0;
//		double uShiftClosenessToReality = uShift*uShift + 70*70;
//		double uShiftClosenessToReality = Math.abs(uShift) + 20;
//		double perfilComp = (50 + sumComp*50/355/countPairs);
		double perfilComp = (20 + sumComp);
		double uVarComp = (3 + uVar + Math.abs(maxU - minU)/10.0 + Math.abs(uShift)/10.0);
		double countPairsComp = countPairs;

		double metricValBase = 100*(maxU - minU)*uShift*sumComp/(uVar*countPairs);
//		double metricVal = 100*perfilComp*countPairsComp*dDisp/(uVarComp*uShiftClosenessToReality + 0.0001);
		double metricVal = 100*perfilComp*countPairs*dDisp/(uVarComp + 0.0001);

		StringBuffer strBuff = new StringBuffer("metrica:\r\n\r\n");
		strBuff.append("base:((maxU - minU), Math.abs(uShift), sumComp, uVar, countPairs)" + (maxU - minU) + ", " + Math.abs(uShift) + ", " + sumComp + ", " + uVar + ", " + countPairs + "::\r\n\r\n");
		strBuff.append("\t -- " + metricValBase + "\r\n\r\n");
		strBuff.append("metric:(perfilComp, countPairsComp, dDisp, uVarComp, uShift)" + perfilComp + ", " + countPairsComp + ", " + dDisp + ", " + uVarComp + ", " + uShift + "::\r\n\r\n");
		strBuff.append("\t -- " + metricVal + "\r\n\r\n");
		return strBuff.toString();
//*/
		return this.getMetric() + "";
	}
}
