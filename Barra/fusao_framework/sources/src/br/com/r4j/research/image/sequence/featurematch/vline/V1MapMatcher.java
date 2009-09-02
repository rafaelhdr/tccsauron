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
public class V1MapMatcher implements LineMapMatcher
{
	private static Log log = LogFactory.getLog(V1MapMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");


	public V1MapMatcher()
	{
	}


	boolean [] arrayMatchesExists = new boolean[100];
	LineProjMatchPair [] arrayMatches = new LineProjMatchPair[100];
	/**
	 * Considera que newLineProjs.arrayProjMeasures está ordenado.
	 *
	 */
	public void matchWithWorldMap(LineExtrationResult newLineProjs, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap)
	{
		// 1 - Realiza o pareamento entre linhas do mapa e projeções realizadas.
		List listExpectedProjs = worldMap.getOrderedExpectedProjections(cameraTrafo, camModel, 1.4);
		log.debug("cameraTrafo: " + cameraTrafo);

		int stride = newLineProjs.arrayProjMeasures.length;
		int sizeExpected = listExpectedProjs.size();
		int size = stride*sizeExpected;
		if (arrayMatches.length < size)
		{
			arrayMatchesExists = new boolean[size + 10];
			arrayMatches = new LineProjMatchPair[size + 10];
		}

		log.debug("expectes projs: " + listExpectedProjs);
		log.debug("measured projs: " + Arrays.toString(newLineProjs.arrayProjMeasures));

		if (listExpectedProjs.size() == 0 || newLineProjs.arrayProjMeasures.length == 0)
		{
			return;
		}

		// 1-a - Cria matriz booleana indicando quais cruzamentos são matches.
		Iterator itExpectedProjs = listExpectedProjs.iterator();
		int expectedIdxFirst = -1, measIdxFirst = -1;
		for (int expectedIdx = 0; expectedIdx < sizeExpected; expectedIdx++)
		{
			VLineRefProj lRefProj = (VLineRefProj) itExpectedProjs.next();
			log.debug("lRefProj: " + lRefProj + ", lRefProj.rLeft, lRefProj.rRight" + lRefProj.getLineRef().rLeft  +", " + lRefProj.getLineRef().rRight + ")");
			VLineProj [] arrayMeasProj = new VLineProj[newLineProjs.arrayProjMeasures.length];
			for (int measIdx = 0; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
			{
				VLineProj lMeasProj = newLineProjs.arrayProjMeasures[measIdx];
				
				// alterar lMeasProj.getPerfil().compare(lRefProj.getLineRef());
				double comp = lMeasProj.getPerfil().compare(lRefProj.getLineRef());
				if (comp > 0.5)
				{
					arrayMatchesExists[measIdx + stride*expectedIdx] = true;
					arrayMatches[measIdx + stride*expectedIdx] = new LineProjMatchPair(lRefProj, lMeasProj, comp);
					log.debug("(" + expectedIdx + ", " + measIdx + ") pair: " + arrayMatches[measIdx + stride*expectedIdx]);
					if (expectedIdxFirst == -1)
					{
						expectedIdxFirst = expectedIdx;
						measIdxFirst = measIdx;
					}
				}
				else
				{
					log.debug(lRefProj + " - " + comp + " - " + lMeasProj + ": stopped by color");
					arrayMatchesExists[measIdx + stride*expectedIdx] = false;
				}
			}
		}
		log.debug("measIdxFirst: " + measIdxFirst + ", stride: " + stride + ", expectedIdxFirst: " + expectedIdxFirst);
		if (measIdxFirst == -1 || expectedIdxFirst == -1)
		{
			return;
		}

		// 1-b - Cria as árvores de associação
		ArrayList listBranchesToGo = new ArrayList();

		Trees tree = new Trees();
		TreeNode treeNodeParent = tree.addRoot(arrayMatches[measIdxFirst + stride*expectedIdxFirst]);
		listBranchesToGo.add(treeNodeParent);
		treeNodeParent.expectedIdx = expectedIdxFirst; treeNodeParent.measIdx = measIdxFirst;

		boolean bFirstTime = true;
		while (listBranchesToGo.size() > 0)
		{
			treeNodeParent = (TreeNode) listBranchesToGo.remove(listBranchesToGo.size() - 1);
			log.debug("(0)root: " + treeNodeParent.expectedIdx + ", " + treeNodeParent.measIdx + ": " + treeNodeParent);

			ArrayList listStack = new ArrayList();
			listStack.add(treeNodeParent);

			if (bFirstTime)
			{
				for (int measIdx = treeNodeParent.measIdx + 1; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
				{
					if (arrayMatchesExists[measIdx + stride*treeNodeParent.expectedIdx])
					{
						TreeNode treeNode = tree.addRoot(arrayMatches[measIdx + stride*treeNodeParent.expectedIdx]);
						log.debug("(1)root: " + treeNodeParent.expectedIdx + ", " + measIdx + ": " + treeNode);
						listBranchesToGo.add(treeNode);
						treeNode.expectedIdx = treeNodeParent.expectedIdx; treeNode.measIdx = measIdx;
					}
				}
			}

			while (listStack.size() > 0)
			{
				treeNodeParent = (TreeNode) listStack.remove(listStack.size() - 1);
				expectedIdxFirst = treeNodeParent.expectedIdx; measIdxFirst = treeNodeParent.measIdx;

				for (int expectedIdx = expectedIdxFirst + 1; expectedIdx < sizeExpected; expectedIdx++)
				{
					boolean bNewNodes = false;
					if (bFirstTime)
					{
						for (int measIdx = 0; measIdx <= measIdxFirst; measIdx++)
						{
							if (arrayMatchesExists[measIdx + stride*expectedIdx])
							{
								TreeNode treeNode = tree.addRoot(arrayMatches[measIdx + stride*expectedIdx]);
								log.debug("(2)root: " + expectedIdx + ", " + measIdx + ": " + treeNode);
								listBranchesToGo.add(treeNode);
								treeNode.expectedIdx = expectedIdx; treeNode.measIdx = measIdx;
							}
						}
					}
					for (int measIdx = measIdxFirst + 1; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
					{
						if (arrayMatchesExists[measIdx + stride*expectedIdx])
						{
							bNewNodes = true;
							TreeNode treeNode = tree.add(arrayMatches[measIdx + stride*expectedIdx], treeNodeParent);
							log.debug("node: " + expectedIdx + ", " + measIdx + ": " + treeNode);
							listStack.add(treeNode);
							treeNode.expectedIdx = expectedIdx; treeNode.measIdx = measIdx;
						}
					}
					if (bNewNodes)
						break;
				}
			}
			bFirstTime = false;
		}

		// 1-c - Cria sequencias de projeções
		ArrayList listProjSequenceSegments = new ArrayList();
		ArrayList listProjSequences = new ArrayList();
		TreeNode [] arrayTreeLeaves = tree.getLeaves();
		for (int i = 0; i < tree.countLeaves(); i++)
		{
			TreeNode nodeBase = arrayTreeLeaves[i];
			TreeNode node = nodeBase;
			log.debug("tree path: " + node);
			log.debug("node.depth: " + node.depth);
			if (node.depth > 1)
			{
				LineProjMatchPair [] arrayPairs = new LineProjMatchPair[node.depth];
				listProjSequences.add(arrayPairs);

				double meanUShift = 0;
				for (int j = 0; j < nodeBase.depth; j++)
				{
					arrayPairs[j] = node.getPair();
					meanUShift += node.getPair().getUShift();
					node = node.getParent();
				}
				meanUShift /= nodeBase.depth;

				log.debug("meanUShift: " + meanUShift);
				node = nodeBase;

				// 1-d - Cria sequencias de pares de projeções (match de 'segmentos')
				int countNodesUsed = 0;
				double meanDShift = 0;
				TreeNode nodeLast = null;
				LineProjMatchPairSegment segPair = null;
				LineProjMatchPairSegment segPairFirst = null;
				for (int j = 0; j < nodeBase.depth; j++)
				{
					// Elimina por variância grande no shift da projeção
					if (Math.abs(arrayPairs[j].getUShift() - meanUShift) > (3 + Math.abs(meanUShift)*0.15))
					{
						log.debug("cleared by u shift: " + arrayPairs[j]);
						log.debug("(arrayPairs[j].getUShift() - meanUShift): " + (arrayPairs[j].getUShift() - meanUShift));
						arrayPairs[j] = null;
					}
					else
					{
						countNodesUsed++;
						if (nodeLast != null)
						{
							if (segPair == null)
							{
								segPair = new LineProjMatchPairSegment(node.getPair(), nodeLast.getPair());
								segPairFirst = segPair;
							}
							else
							{
								segPair.next = new LineProjMatchPairSegment(node.getPair(), nodeLast.getPair());
								segPair = segPair.next;
							}
							meanDShift += segPair.getDShift();
						}
						nodeLast = node;
					}
					node = node.getParent();
				}
				log.debug("countNodesUsed: " + countNodesUsed);
				log.debug("segPairFirst: " + segPairFirst);

				if (countNodesUsed > 1)
				{
					// Daria pra eliminar esse negocio do dShift?
					// O uMean já não cobre?
					meanDShift /= countNodesUsed;
					segPair = segPairFirst;
					LineProjMatchPairSegment segPairLast = segPairFirst;
					int countSegs = 0;
					while (segPair != null)
					{
						if (Math.abs(segPair.getDShift() - meanDShift) > 0.1)
						{
							log.debug("cleared by d shift: " + segPair);
							if (segPairLast == segPair)
							{
								segPairFirst = segPair.next;
								segPairLast = segPairFirst;
							}
							else
							{
								segPairLast.next = segPair.next;
							}
						}
						else
						{
							segPairLast = segPair;
							countSegs++;
						}

						segPair = segPair.next;
					}
					if (segPairFirst != null)
					{
						listProjSequenceSegments.add(segPairFirst);
						log.debug("segPairFirst(CLEARED): " + segPairFirst);
					}
				}
			}
		}

		// 1-e - Calcula os valores de comparação e determina a melhor sequecência.
		Iterator itProjSequenceSegments = listProjSequenceSegments.iterator();
		TreeMap mapProjSequenceSegmentMetrics = new TreeMap();
		while (itProjSequenceSegments.hasNext())
		{
			LineProjMatchPairSegment segPairFirst = (LineProjMatchPairSegment) itProjSequenceSegments.next();
			LineProjMatchPairSegment segPair = segPairFirst;
			LineProjMatchPairSegment segPairLast = null;
			log.debug("1-e: segPairFirst = " + segPairFirst);
			double uShift = 0, sumComp = 1, minU = 10000, maxU = -10000;
			int countSegs = 0, countPairs = 0;
			while (segPair != null)
			{
				if (segPairLast == null)
				{
					log.debug("\t (if) segPair.getPair1().getUShift() = " + segPair.getPair1().getUShift());
					uShift += segPair.getPair1().getUShift();
					sumComp *= segPair.getPair1().getComp();
					if (minU > segPair.getPair1().getMeasProj().getU())
						minU = segPair.getPair1().getMeasProj().getU();
					if (maxU < segPair.getPair1().getMeasProj().getU())
						maxU = segPair.getPair1().getMeasProj().getU();
					countPairs++;
				}
				uShift += segPair.getPair2().getUShift();
				log.debug("\t segPair.getPair2().getUShift() = " + segPair.getPair2().getUShift());
				sumComp *= segPair.getPair2().getComp();
				if (minU > segPair.getPair2().getMeasProj().getU())
					minU = segPair.getPair2().getMeasProj().getU();
				if (maxU < segPair.getPair2().getMeasProj().getU())
					maxU = segPair.getPair2().getMeasProj().getU();
				countSegs++;
				countPairs++;

				segPairLast = segPair;
				segPair = segPair.next;
			}
			if (countPairs > 0)
				uShift /= countPairs;

			double uVar = 0;
			segPair = segPairFirst; segPairLast = null;
			while (segPair != null)
			{
				if (segPairLast == null)
				{
					uVar += (segPair.getPair1().getUShift() - uShift)*(segPair.getPair1().getUShift() - uShift);
					log.debug("\t (segPair.getPair1().getUShift() - uShift)*(segPair.getPair1().getUShift() - uShift) = " + (segPair.getPair1().getUShift() - uShift)*(segPair.getPair1().getUShift() - uShift));
				}
				uVar += (segPair.getPair2().getUShift() - uShift)*(segPair.getPair2().getUShift() - uShift);
				log.debug("\t (segPair.getPair2().getUShift() - uShift)*(segPair.getPair2().getUShift() - uShift) = " + (segPair.getPair2().getUShift() - uShift)*(segPair.getPair2().getUShift() - uShift));

				segPairLast = segPair;
				segPair = segPair.next;
			}
			if (countPairs > 1)
				uVar /= (countPairs - 1);

			log.debug("uVar: " + uVar + ", uShift: " + uShift + ", minU: " + minU + ", maxU: " + maxU + ", countPairs: " + countPairs);

			LineProjSequenceSegmentMetric metric = new LineProjSequenceSegmentMetric(uShift, uVar, minU, maxU, sumComp, countPairs, listExpectedProjs.size());
			mapProjSequenceSegmentMetrics.put(metric, segPairFirst);
			log.debug("segPairFirst(metric): " + segPairFirst + "(" + metric + ")");
		}

		////////
		//// 2 - Adiciona ao VLineMap as associações entre as projeções (VLines associadas) medidas e linhas do mapa.
		if (mapProjSequenceSegmentMetrics.keySet().iterator().hasNext())
		{
			LineProjSequenceSegmentMetric metric = (LineProjSequenceSegmentMetric) mapProjSequenceSegmentMetrics.keySet().iterator().next();
			LineProjMatchPairSegment segPairFirst = (LineProjMatchPairSegment) mapProjSequenceSegmentMetrics.get(metric);

			log.debug("segPairFirst(selected): " + segPairFirst);

			LineProjMatchPairSegment segPair = segPairFirst;
			LineProjMatchPairSegment segPairLast = null;
			while (segPair != null)
			{
/*
				if (segPairLast == null) // && segPair.getPair1() != segPairLast.getPair2())
				{
					VLineRef lineRef = segPair.getPair1().getRefProj().getLineRef();
					VLineProj lMeasProj = segPair.getPair1().getMeasProj();
					VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
					lineMap.setWorldMapMatchByProjection(line, lineRef);

					log.debug("(di)match |-> lineRef: " + lineRef + ", line: " + line);
				}
//*/

				VLineRef lineRef = segPair.getPair2().getRefProj().getLineRef();
				VLineProj lMeasProj = segPair.getPair2().getMeasProj();
				VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
				lineMap.setWorldMapMatchByProjection(line, lineRef);

				log.debug("match |-> lineRef: " + lineRef + ", line: " + line);

				segPairLast = segPair;
				segPair = segPair.next;
			}
			if (segPairLast != null) // && segPair.getPair1() != segPairLast.getPair2())
			{
				VLineRef lineRef = segPairLast.getPair1().getRefProj().getLineRef();
				VLineProj lMeasProj = segPairLast.getPair1().getMeasProj();
				VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
				lineMap.setWorldMapMatchByProjection(line, lineRef);

				log.debug("(duu)match |-> lineRef: " + lineRef + ", line: " + line);
			}
		}

		////////
		//// 3 - No VLineMap, verifica se a Vline já foi associada a mesma linah do mapa. Se sim,e se por três vezes, a projeção
		// passa a ser usada como medição, e o esperado é calculado usando a linha do mapa e  posição do robo.
	}
}


