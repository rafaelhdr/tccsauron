/*
 * Created on Dec 11, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.gui;

import br.com.r4j.gui.RendererListener;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 * @modelguid {767B414A-E512-4CFD-ACAC-2947608D759E}
 */
public class RendererListenerHolder
{
	/** @modelguid {7D2B4C12-D77E-49E3-B3BF-95B675F2B473} */
	private RendererListener rndrList = null;

	/**
	 * @param listener
	 * @modelguid {843DBF82-CA94-4018-9162-10661D1AB666}
	 */
	public RendererListenerHolder(RendererListener listener)
	{
		rndrList = listener;
	}


	/**
	 * @return
	 * @modelguid {F2A20F46-1EA9-4384-89BB-A097CC50149A}
	 */
	public RendererListener getRendererListener()
	{
		return rndrList;
	}


	/** @modelguid {E36010B9-4F5C-4250-BCB5-322FDEDA408C} */
	public String toString()
	{
		return rndrList.getName();
	}
}
