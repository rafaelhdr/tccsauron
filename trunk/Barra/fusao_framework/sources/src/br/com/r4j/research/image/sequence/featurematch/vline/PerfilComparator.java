package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.VLineMap;


public class PerfilComparator implements Comparable
{
	public double realComparsionValue;

	public double comparsion;
	public double comparsionFirstHalf;
	public double comparsionSecondHalf;
	public int desempate = 0;
	public int matchedBand = VLineMap.NO_BAND;

	public PerfilComparator(double comparsion, double comparsionFirstHalf, double comparsionSecondHalf, int desempate)
	{
		this.comparsion = comparsion;
		this.comparsionFirstHalf = comparsionFirstHalf;
		this.comparsionSecondHalf = comparsionSecondHalf;
		this.desempate = desempate;

		double bandDiff = Math.abs(comparsionSecondHalf - comparsionFirstHalf);

		if (bandDiff > 0.225*comparsion)
		{
			if (comparsionSecondHalf > comparsionFirstHalf)
			{
				realComparsionValue = comparsionSecondHalf*0.9;
				matchedBand = VLineMap.SECOND_BAND;
			}
			else
			{
				realComparsionValue = comparsionFirstHalf*0.9;
				matchedBand = VLineMap.FIRST_BAND;
			}
		}
		else
		{
			realComparsionValue = comparsion;
			matchedBand = VLineMap.BOTH_BANDS;
		}
	}


	public double getComparisonValue()
	{
		return realComparsionValue;
	}


	public int getMatchedBand()
	{
		return matchedBand;
	}

	
	public int compareTo(Object o)
	{
		PerfilComparator compa = (PerfilComparator) o;
		double diff = realComparsionValue - compa.getComparisonValue();
		if (diff == 0)
			return desempate - compa.desempate;
		else if (diff > 0)
			return -1;
		else
			return 1;
	}

	public String toString()
	{
		return "" + getComparisonValue();
	}

}
