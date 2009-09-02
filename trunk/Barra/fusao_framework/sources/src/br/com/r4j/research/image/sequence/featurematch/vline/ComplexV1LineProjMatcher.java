package br.com.r4j.research.image.sequence.featurematch.vline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;
import java.text.*;

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
public class ComplexV1LineProjMatcher implements LineProjMatcher
{
	private static Log log = LogFactory.getLog(ComplexV1LineProjMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");
	private static Log logVisionPairsCount = LogFactory.getLog("pares_visao");
	private static Log logVisionPairs = LogFactory.getLog("pares_visao_u");
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private double hashVectorTreshold = 50;


	public ComplexV1LineProjMatcher()
	{
	}
	

	public void setHashVectorTreshold(double hashVectorTreshold)	{this.hashVectorTreshold = hashVectorTreshold;}

	public void performMatches(LineExtrationResult lineExtrResult, PairedLines pairedLines, LineMatchingResult result, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap, DifferenceSimpleMatchingPhaseMatcher callback)
	{
		log.debug("PERFORMMATCHES:ENTERED");
		long timeTaken = System.currentTimeMillis();
		result.arrayProjMeasuresLines = new VLine[lineExtrResult.arrayProjMeasures.length];
		ArrayList listOldies = new ArrayList();

		for (int i = 0; i < pairedLines.arrayTreeMapLinks.length; i++)
		{
			VLineProj lineProj = lineMap.getLastMeasuredProjection(pairedLines.arrayProjLastMeasuresLines[i]);
			String strVals = "u = " + lineProj.getU() + ": ";
			TreeMap treeCorrs = pairedLines.arrayTreeMapLinks[i];
			if (treeCorrs != null)
			{
				Iterator it = treeCorrs.keySet().iterator();
				while (it.hasNext())
				{
					PerfilComparator compa = (PerfilComparator) it.next();
					int keys = ((Integer) treeCorrs.get(compa)).intValue();
					int idxNewMeas = keys&0xFFFF;
					VLineProj lineProj2 = lineExtrResult.arrayProjMeasures[idxNewMeas];
					strVals += (compa.getComparisonValue()) + ":[" + lineProj2.getU() + "], ";
				}
			}
			log.debug(strVals);
		}


		// Como primeira implementa��o, a estrutura de diferen�as � refeita a cada itera��o,
		// mas pode mudar para ser amis eficiente, deixando o primeiro loop interno (pesado)
		// apenas como inicializa��o.
		int desempate = 0;
		int countCount = 0;
		String strPairs = "";
		while (true)
		{
			// Inicia a estrutura de diferen�as inserindo as maiores diferen�as entre os maiores
			// corrs para cada �ltima proj, e o valor dos �ndices associados a �ltima e nova
			// projs.
			TreeMap mapDiffs = new TreeMap();
			log.debug("performmatches:pairedLines.arrayTreeMapLinks.length = " + pairedLines.arrayTreeMapLinks.length);
			for (int i = 0; i < pairedLines.arrayTreeMapLinks.length; i++)
			{
				TreeMap treeCorrs = pairedLines.arrayTreeMapLinks[i];

				// Se a linha ainda n�o foi matched, e se ainda tem algum potencial candidato.
				if (!pairedLines.arrayProjLastMeasuresAlreadyUsed[i] && treeCorrs != null && treeCorrs.size() > 0)
				{
					// Pega o primeiro candidato
					double valBigger = 0;
					PerfilComparator keyBigger = null;
					while (treeCorrs.size() > 0)
					{
						PerfilComparator compa = (PerfilComparator) treeCorrs.firstKey();
						int keys = ((Integer) treeCorrs.get(compa)).intValue();
						int idxNewMeas = keys&0xFFFF;
						if (!pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas])
						{
							valBigger = compa.getComparisonValue();
							keyBigger = compa;
							break;
						}
						else
							treeCorrs.remove(compa);
					}

					// Se tem um primeiro candidato
					if (keyBigger != null)
					{
						listOldies.clear();
						
						// Pega o segundo candidato (n�o checa se tem ou n�o depois ...)
						double valNext = hashVectorTreshold;
						Iterator itCorrKeys = treeCorrs.keySet().iterator(); itCorrKeys.next(); 
						while (itCorrKeys.hasNext())
						{
							PerfilComparator compa = (PerfilComparator) itCorrKeys.next();
							int keys = ((Integer) treeCorrs.get(compa)).intValue();
							int idxNewMeas = keys&0xFFFF;
							if (!pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas])
							{
								valNext = compa.getComparisonValue();
								break;
							}
							else
								listOldies.add(compa);
						}

						// P�e na lista ordenada os �ndices do potencial match associado com a maior differen�a.
						mapDiffs.put(new DoubleCompa(-valBigger*(valBigger - valNext), keyBigger), treeCorrs.get(keyBigger));

						// Limpa candidatos que n�o podem masi ser usados
						if (listOldies.size() > 0)
						{
							Iterator itJunk = listOldies.iterator();
							while (itJunk.hasNext())
								treeCorrs.remove(itJunk.next());
						}
					}
					else
					{
						pairedLines.arrayTreeMapLinks[i] = null;
					}
				}
			}
			if (mapDiffs.size() > 0)
			{
				// Teoricamente, n�o precisa desse loop, mas como pode ter algum erro, assim fica mais f�cil de pegar.
				while (true)
				{
					DoubleCompa keyBigger = (DoubleCompa) mapDiffs.firstKey();
					PerfilComparator compPerf = keyBigger.getPerfilComparator();

					int keys = ((Integer) mapDiffs.get(keyBigger)).intValue();
					int idxLastMeas = (keys>>16)&0xFFFF;
					int idxNewMeas = keys&0xFFFF;
					if (!pairedLines.arrayProjLastMeasuresAlreadyUsed[idxLastMeas] && !pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas])
					{
						LineMatch lineMatch = new LineMatch();
						lineMatch.line = pairedLines.arrayProjLastMeasuresLines[idxLastMeas];
						lineMatch.projNew = lineExtrResult.arrayProjMeasures[idxNewMeas];
						lineMatch.projLast = lineMap.getLastMeasuredProjection(pairedLines.arrayProjLastMeasuresLines[idxLastMeas]);
						lineMatch.matchedBand = compPerf.getMatchedBand();

						log.debug("results:comparsion = " + keyBigger + ",u last: " + lineMatch.projLast.getU() + ",u: " + lineMatch.projNew.getU() + ":used");
						log.debug("lineMatch.matchedBand: " + lineMatch.matchedBand);
						pairedLines.arrayProjLastMeasuresAlreadyUsed[idxLastMeas] = true;
						pairedLines.arrayProjMeasuresAlreadyUsed[idxNewMeas] = true;

//						lineMatch.line = lineMatch;
						callback.projectionMatched(lineMatch, lineExtrResult.arrayProjMeasures[idxNewMeas].getPerfil(), cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar);

						result.arrayProjMeasuresLines[idxNewMeas] = lineMatch.line;
						result.listLineMatches.add(lineMatch);

						if (strPairs.length() > 2)
							strPairs += ", ";
						strPairs += "(" + numFrmt.format(lineMatch.projLast.getU()) + ", " + numFrmt.format(lineMatch.projNew.getU()) + ")";
						countCount++;

						break;
					}
					else
					{
						log.debug("ERRO DE L�GICA 1");
						mapDiffs.remove(keyBigger);
					}
				}
			}
			// Sai fora, nenhum mach a mais ...
			else
			{
				break;
			}
		}
		log.debug("performMatches: timeTaken = " + (System.currentTimeMillis() - timeTaken));
		logVisionPairsCount.debug(countCount + ";");
		logVisionPairs.debug(strPairs + ";");
	}
}

