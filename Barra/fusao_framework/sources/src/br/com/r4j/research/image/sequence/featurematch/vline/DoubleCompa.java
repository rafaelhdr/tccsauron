package br.com.r4j.research.image.sequence.featurematch.vline;



public class DoubleCompa implements Comparable
{
	public double dVal = 0;
	public PerfilComparator desempate = null;

	public DoubleCompa(double dVal, PerfilComparator desempate)
	{
		this.dVal = dVal;
		this.desempate = desempate;
	}
	
	public int compareTo(Object o)
	{
		DoubleCompa	compa = (DoubleCompa) o;
		double diff = dVal - compa.dVal;
		if (diff == 0)
			return desempate.compareTo(compa.desempate);
		else if (diff < 0)
			return -1;
		else
			return 1;
	}


	public PerfilComparator getPerfilComparator()
	{
		return desempate;
	}
	
	public String toString()
	{
		return "" + dVal;
	}

}

