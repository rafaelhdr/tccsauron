package br.com.r4j.robosim;

import javax.swing.Icon;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.robosim.estimator.EstimatesEngine;


public interface StatsComponent
{
	public void setEstimatesEngine(EstimatesEngine estEng);

	public BlockUI getUI();

	public String getShortName();
	
	public Icon getIcon();

	public String getDescription();
}
