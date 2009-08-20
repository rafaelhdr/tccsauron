package br.com.r4j.robosim.estimator.provider;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.collections.iterators.IteratorChain;


public class EstimationComponentsInfo
{
	protected String name = null;
	protected EstimatorInfo est = null;
	protected ArrayList listSens = null;
	protected DynamicModelInfo dynModel = null;


	public EstimationComponentsInfo(String name)
	{
		this.name = name;
		listSens = new ArrayList();
	}


	public EstimatorInfo getEstimator() {return this.est;}
	public void setEstimator(EstimatorInfo est) {this.est = est;}


	public List getSensorModels() {return listSens;}
	public void addSensorModel(SensorModelInfo sensModel) {listSens.add(sensModel);}

	public DynamicModelInfo getDynamicModel() {return this.dynModel;}
	public void setDynamicModel(DynamicModelInfo dynModel) {this.dynModel = dynModel;}


	public String getName()
	{
		return name;
	}

	
	public String toString()
	{
		return name;
	}


	public EstimationComponentsInfo getCopy(String strName)
	{
		EstimationComponentsInfo snsModelCopy = new EstimationComponentsInfo(strName);

		snsModelCopy.est = est.getCopy();
		
		snsModelCopy.listSens = new ArrayList();
		Iterator itSns =  listSens.iterator();
		while (itSns.hasNext())
		{
			SensorModelInfo snsInfo = (SensorModelInfo) itSns.next();
			snsModelCopy.listSens.add(snsInfo.getCopy()); 
		}
		snsModelCopy.dynModel = dynModel.getCopy();

		return snsModelCopy;
	}
	
	
	public boolean equals(Object o)
	{
		if (o instanceof EstimationComponentsInfo)
			return name.equals(((EstimationComponentsInfo) o).getName());
		else
			return false;
	}
	
	
	public int hashCode()
	{
		return name.hashCode();
	}
}
