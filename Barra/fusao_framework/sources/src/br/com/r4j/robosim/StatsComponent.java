package br.com.r4j.robosim;

import javax.swing.Icon;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.EstimationStatisticsEvent;


/** @modelguid {3D20E6A6-7C60-472F-BA3E-C14BD0B4E8A3} */
public interface StatsComponent
{
	/** @modelguid {867B1F59-756A-4E36-83EF-2872C4D23DE5} */
	public void setEstimatesEngine(EstimatesEngine estEng);

	/** @modelguid {0EE568D6-4EB7-4E24-8AEB-6E1541E99E51} */
	public BlockUI getUI();

	/** @modelguid {D77965EF-D32A-46E8-8A5C-AA5214971122} */
	public String getShortName();
	
	/** @modelguid {D7F2282B-57B8-493A-B39F-1CB5870CE774} */
	public Icon getIcon();

	/** @modelguid {D6E92DAF-422B-457C-81BF-55CCF47C6EB4} */
	public String getDescription();

	public void estimationPerformed(EstimationStatisticsEvent e);
}
