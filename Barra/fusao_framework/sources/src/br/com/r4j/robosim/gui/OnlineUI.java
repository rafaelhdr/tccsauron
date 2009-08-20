/*
 * Created on Dec 12, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.gui;

import java.util.Map;

import javax.swing.JComponent;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public interface OnlineUI
{
	public void setRendererInfos(Map mapRendererInfos);


	public Map getRendererInfos();


	public void resetUI();
	
	
	public JComponent getComponent();
}
