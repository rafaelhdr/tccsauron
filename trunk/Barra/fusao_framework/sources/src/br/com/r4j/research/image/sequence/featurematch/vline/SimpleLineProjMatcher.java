package br.com.r4j.research.image.sequence.featurematch.vline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.vline.*;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.commons.util.*;


/**
 *
 */
public class SimpleLineProjMatcher implements LineProjMatcher
{
	private static Log log = LogFactory.getLog(SimpleLineProjMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");

	private double hashVectorTreshold = 50;


	public SimpleLineProjMatcher()
	{
	}


	public void setHashVectorTreshold(double hashVectorTreshold)	{this.hashVectorTreshold = hashVectorTreshold;}


	public void performMatches(LineExtrationResult lineExtrResult, PairedLines pairedLines, LineMatchingResult result, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap, DifferenceSimpleMatchingPhaseMatcher callback)
	{
		long timeTaken = System.currentTimeMillis();
		Iterator itMatches = pairedLines.treeMapLinks.keySet().iterator();
		log.debug("treeMapLinks.keySet().size() = " + pairedLines.treeMapLinks.keySet().size());
		while (itMatches.hasNext())
		{
			Object oKey = itMatches.next();
			Integer intKeys = (Integer) pairedLines.treeMapLinks.get(oKey);
			int keys = intKeys.intValue();
			int idxLastMeas = (keys>>16)&0xFFFF;
			int idxNewMeas = keys&0xFFFF;

			if (!pairedLines.arrayProjLastMeasuresAlreadyUsed[idxLastMeas] && !pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas])
			{
				LineMatch lineMatch = new LineMatch();
				result.listLineMatches.add(lineMatch);
				lineMatch.line = pairedLines.arrayProjLastMeasuresLines[idxLastMeas];
				lineMatch.projNew = lineExtrResult.arrayProjMeasures[idxNewMeas];
				lineMatch.projLast = lineMap.getLastMeasuredProjection(pairedLines.arrayProjLastMeasuresLines[idxLastMeas]);

				log.debug("results:comparsion = " + oKey + ",u last: " + lineMatch.projLast.getU() + ",u: " + lineMatch.projNew.getU() + ":used");
				pairedLines.arrayProjLastMeasuresAlreadyUsed[idxLastMeas] = true;
				pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas] = true;
				result.arrayProjMeasuresLines[idxNewMeas] = lineMatch.line;

				callback.projectionMatched(lineMatch, lineExtrResult.arrayProjMeasures[idxNewMeas].getPerfil(), cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar);
			}
			else
			{
				log.debug("results:comparsion = " + oKey + ",u last: " + lineMap.getLastMeasuredProjection(pairedLines.arrayProjLastMeasuresLines[idxLastMeas]).getU() + ",u: " + lineExtrResult.arrayProjMeasures[idxNewMeas].getU() + ":not used");
			}
		}
		log.debug("performMatches: timeTaken = " + (System.currentTimeMillis() - timeTaken));
	}
}


