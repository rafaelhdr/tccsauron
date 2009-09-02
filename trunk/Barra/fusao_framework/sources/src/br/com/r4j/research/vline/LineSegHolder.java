package br.com.r4j.research.vline;

import java.util.ArrayList;


public class LineSegHolder implements Comparable
{
//	public TreeSet setPairs = new TreeSet();
	public ArrayList setPairs = new ArrayList(5);

	public LineSegment lineSeg = null;

	public LineSegHolder(LineSegment lineSeg)
	{
		this.lineSeg = lineSeg;
	}


	public int compareTo(Object o)
	{
		if (o instanceof LineSegment)
			return lineSeg.compareTo(o);
		else
			return lineSeg.compareTo(((LineSegHolder) o).lineSeg);
	}


	public String toString()
	{
		return "(hld: " + lineSeg.toString() + ")";
	}
}
