package br.com.r4j.research.image.sequence.featurematch.vline;

import br.com.r4j.research.vline.*;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


public class Trees
{
	private static Log log = LogFactory.getLog(Trees.class.getName());

	private TreeNode [] roots = null; 
	private TreeNode [] leaves = null; 

	private int freeRootIdx = 0;
	private int freeLeaveIdx = 0;


	public Trees()
	{
		roots = new TreeNode[1000]; 
		leaves = new TreeNode[5000];
	}


	public TreeNode addRoot(LineProjMatchPair matchPair)
	{
		TreeNode node = new TreeNode(matchPair);
		node.leaveIdx = freeLeaveIdx;
		node.depth = 1;
		roots[freeRootIdx++] = node; 
		leaves[freeLeaveIdx++] = node;

		return node;
	}


	public TreeNode add(LineProjMatchPair matchPair, TreeNode treeNodeParent)
	{
		TreeNode node = new TreeNode(matchPair);
		treeNodeParent.addChild(node);
		node.setParent(treeNodeParent);
		node.depth = treeNodeParent.depth + 1;
		if (treeNodeParent.leaveIdx != -1)
		{
			node.leaveIdx = treeNodeParent.leaveIdx;
			treeNodeParent.leaveIdx = -1;
		}
		else
			node.leaveIdx = freeLeaveIdx++;
		leaves[node.leaveIdx] = node;

		return node;

	}


	public TreeNode [] getLeaves()
	{
		return leaves;
	}

	
	public int  countLeaves()
	{
		return freeLeaveIdx;
	}

 
 	public String toString()
	{
		return "tree:?";
	}
}
