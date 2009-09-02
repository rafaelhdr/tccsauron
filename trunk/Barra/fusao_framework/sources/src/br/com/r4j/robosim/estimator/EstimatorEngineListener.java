/*
 * Created on Jan 2, 2006
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public interface EstimatorEngineListener
{
	public void estimationIterationPerformed(AbstractDoubleVector [] arrayRealPoses, AbstractDoubleVector [][] arrayMeans, AbstractDoubleSquareMatrix [][] arrayCovars, int itCount, int estCount, int stepCount);
}
