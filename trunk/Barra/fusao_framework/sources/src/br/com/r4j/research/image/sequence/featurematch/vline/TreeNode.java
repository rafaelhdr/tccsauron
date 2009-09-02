package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.*;
import java.util.*;
import java.text.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


public class TreeNode
{
	private static Log log = LogFactory.getLog(TreeNode.class.getName());

	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private LineProjMatchPair matchPair = null;
	private TreeNode parent = null;
	private ArrayList listChilds = null;

	public int depth = -1;
	public int leaveIdx = -1;
	public int expectedIdx = -1; 
	public int measIdx = -1;


	public TreeNode(LineProjMatchPair matchPair)
	{
		this.matchPair = matchPair;
		listChilds = new ArrayList();
	}


	public LineProjMatchPair getPair()
	{
		return matchPair;
	}


	public TreeNode getParent()
	{
		return parent;
	}


	public void setParent(TreeNode parent)
	{
		this.parent = parent;
	}


	public void addChild(TreeNode child)
	{
		listChilds.add(child);
	}


	public String toString()
	{
		String strNode = "["+numFrmt.format(matchPair.getRefProj().getU())+", "+numFrmt.format(matchPair.getMeasProj().getU())+"]";
		if (parent != null)
			return strNode + "->" + parent;
		else
			return strNode;
	}
}
