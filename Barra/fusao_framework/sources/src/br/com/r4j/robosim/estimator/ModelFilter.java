/*
 * Created on Dec 4, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.estimator;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 * @modelguid {56051F11-23B9-4B53-A84D-A26638BF94CC}
 */
public interface ModelFilter
{
	/** @modelguid {A4B3680E-8D91-4F05-B8BC-66155CE52321} */
	public boolean canUseDynamicModel(DynamicModel dynModel);


	/** @modelguid {D5E7D944-48FB-44F6-B94F-E252AAFD0A19} */
	public boolean canUseSensorModel(SensorModel sensModel);

}
