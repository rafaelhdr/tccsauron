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
 * @modelguid {C77E055E-CE44-4998-9A51-45798EC64DB8}
 */
public interface OnlineUI
{
	/** @modelguid {9BD5FC9A-AFBE-41C3-9D1F-61F6AAFA2FCB} */
	public void setRendererInfos(Map mapRendererInfos);


	/** @modelguid {423746C7-C588-4830-B8F2-181B40000406} */
	public Map getRendererInfos();


	/** @modelguid {AAAF1A5D-F65C-461D-8937-59AD16CA0584} */
	public void resetUI();
	
	
	/** @modelguid {ECC6973E-8522-4FFA-8B92-3337A08F8E9A} */
	public JComponent getComponent();
}
