/*
 * Created on Dec 16, 2005
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
 * @modelguid {15E5BD57-CBE8-40D5-A9CD-2036DC608B88}
 */
public interface EstimateObserver
{
	/** @modelguid {1C8491A6-6DA0-4A0F-816B-71DD5D3D3BEA} */
	public void newEstimate(AbstractDoubleVector mean, AbstractDoubleSquareMatrix covar);

}
