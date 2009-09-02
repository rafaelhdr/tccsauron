

package br.com.r4j.research.vline;

import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/** 
 *
 */
public class LineSegmentCollection extends LineSegment
{
	protected static Log log = LogFactory.getLog(LineSegmentCollection.class.getName());

	public ArrayList listSegs = new ArrayList();


	public LineSegmentCollection(LineSegment ls)
	{
		this.addSeg(ls);
	}


	public LineSegmentCollection(LineSegHolder ls)
	{
		this.addSeg(ls.lineSeg);
	}


	public void addSeg(LineSegment ls)
	{
		imgHeight = ls.imgHeight;
		if (yIni > ls.yIni) {yIni = ls.yIni; xIni = ls.xIni;}
		if (yEnd < ls.yEnd) {yEnd = ls.yEnd; xEnd = ls.xEnd;}

		xMidle = -66666;
		if (ls.getClass() == LineSegmentCollection.class)
		{
			LineSegmentCollection lsColl = (LineSegmentCollection) ls;

//			xModel = (listSegs.size()*xModel + ls.xModel*lsColl.listSegs.size())/(lsColl.listSegs.size() + listSegs.size());
//			mModel = (listSegs.size()*mModel + ls.mModel*lsColl.listSegs.size())/(lsColl.listSegs.size() + listSegs.size());

			xModel = (sizeSeg*xModel + ls.xModel*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));
			mModel = (sizeSeg*mModel + ls.mModel*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));

			xMidMean = (xMidMean*sizeSeg + ls.xMidMean*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));
			xMidMean2 = (xMidMean2*sizeSeg + ls.xMidMean2*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));

			listSegs.addAll(lsColl.listSegs);
		}
		else
		{
//			xModel = (listSegs.size()*xModel + ls.xModel)/ (1 + listSegs.size());
//			mModel = (listSegs.size()*mModel + ls.mModel)/ (1 + listSegs.size());

			xModel = (sizeSeg*xModel + ls.xModel*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));
			mModel = (sizeSeg*mModel + ls.mModel*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));

			xMidMean = (xMidMean*sizeSeg + ls.xMidMean*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));
			xMidMean2 = (xMidMean2*sizeSeg + ls.xMidMean2*ls.sizeSeg)/(1.0*(ls.sizeSeg + sizeSeg));

			listSegs.add(ls);
		}

//		log.debug("addSeg: sizeSeg: " + sizeSeg + ", ls.sizeSeg: " + ls.sizeSeg + ", xModel = " + xModel + ", mModel = " + mModel + ", ls.xModel = " + ls.xModel + ", ls.mModel = " + ls.mModel);

		sizeSeg += ls.sizeSeg;
	}


	public boolean superimpose(LineSegment ls)
	{
		if (ls.getClass() == LineSegmentCollection.class)
		{
			LineSegmentCollection lsColl = (LineSegmentCollection) ls;

			for (int j = 0; j < lsColl.listSegs.size(); j++)
			{
				ls = (LineSegment) lsColl.listSegs.get(j);
				for (int i = 0; i < listSegs.size(); i++)
				{
					LineSegment lsNext = (LineSegment) listSegs.get(i);
					if (lsNext.superimpose(ls))
						return true;
				}
			}
		}
		else
		{
			for (int i = 0; i < listSegs.size(); i++)
			{
				LineSegment lsNext = (LineSegment) listSegs.get(i);
				if (lsNext.superimpose(ls))
					return true;
			}
		}
		return false;
	}
}
