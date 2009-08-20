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
 */
public class RendererListenerHolder
{
	private RendererListener rndrList = null;

	/**
	 * @param listener
	 */
	public RendererListenerHolder(RendererListener listener)
	{
		rndrList = listener;
	}


	/**
	 * @return
	 */
	public RendererListener getRendererListener()
	{
		return rndrList;
	}


	public String toString()
	{
		return rndrList.getName();
	}
}
