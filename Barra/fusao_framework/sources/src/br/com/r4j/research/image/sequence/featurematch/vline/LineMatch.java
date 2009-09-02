package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.VLine;
import br.com.r4j.research.vline.VLineMap;
import br.com.r4j.research.vline.VLineProj;


public class LineMatch
{
	VLine line = null;
	VLineProj projNew = null;
	VLineProj projLast = null;
	int matchedBand = VLineMap.NO_BAND;
}
