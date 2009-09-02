

package br.com.r4j.research.vline;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/** 
 * Passos de execução:
 *
 *
 */
public class LineSegment
{
	protected static Log log = LogFactory.getLog(LineSegment.class.getName());

	public int xIni = 99999;
	public int yIni = 99999;

	public int xEnd = 0;
	public int yEnd = 0;

	public int sizeSeg = 0;

	public int imgHeight = 0;


	public double xModel = 0;
	public double mModel = 0;

	public double xMidMean = 0;
	public double xMidMean2 = 0;

	public double xMidle = -66666;


	public boolean superimpose(LineSegment ls)
	{
		if (ls.getClass() == LineSegmentCollection.class)
			return ls.superimpose(this);

		return !(yIni > ls.yEnd || ls.yIni > yEnd);
	}


	public double getXMidle()
	{
		if (xMidle == -66666)
		{
//			xMidle = (xIni + xEnd)/2;
			xMidle = imgHeight*mModel/2 + xModel;
//			log.debug("xMidle: " + xMidle + ", mModel: " + mModel + ", xModel = " + xModel);
		}
		return xMidle;
	}


	public double getXMidleOfSegment()
	{
//		log.debug("getXMidleOfSegment: (xIni + xEnd)*1.0/2.0: " + (xIni + xEnd)*1.0/2.0 + ", getXMidle(): " + getXMidle() + ", ((yEnd + yIni)*mModel/2 + xModel): " + ((yEnd + yIni)*mModel/2 + xModel));
//		return (xIni + xEnd)*1.0/2.0;
		return (yEnd + yIni)*mModel/2 + xModel;
	}


	public int compareTo(Object o)
	{
		if (xMidle == -66666)
		{
//			xMidle = (xIni + xEnd)/2;
			xMidle = imgHeight*mModel/2 + xModel;
//			log.debug("xMidle: " + xMidle + ", mModel: " + mModel + ", xModel = " + xModel);
		}

		LineSegment lineSeg = (LineSegment) o;
		if (lineSeg.xMidle == -66666)
		{
//			lineSeg.xMidle = (lineSeg.xIni + lineSeg.xEnd)/2;
			lineSeg.xMidle = imgHeight*lineSeg.mModel/2 + lineSeg.xModel;
//			log.debug("lineSeg.xMidle: " + lineSeg.xMidle + ", mModel: " + lineSeg.mModel + ", xModel = " + lineSeg.xModel);
		}

		int comp = (int) ((xMidle - lineSeg.xMidle)*10);
		if (comp != 0)
			return comp;
		else
		{
			comp = xIni - lineSeg.xIni;
			if (comp != 0)
				return comp;
			else
			{
				comp = (int) (100000*(mModel - lineSeg.mModel));
				if (comp != 0)
					return comp;
				else
				{
					comp = (int) (sizeSeg - lineSeg.sizeSeg);
					if (comp != 0)
						return comp;
					else
						return yIni - lineSeg.yIni;
				}
			}
		}
	}


	public String toString()
	{
		return "(" + xIni + ", " + yIni + "), [" + ((float) ((int) (mModel*1000)))/1000.0 + ", " + xModel + "], " + sizeSeg + ", " + xMidle;
	}
}
