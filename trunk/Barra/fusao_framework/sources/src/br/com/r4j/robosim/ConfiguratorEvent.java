package br.com.r4j.robosim;



/** @modelguid {FB24B31A-745A-4C81-A346-5397C765B0B7} */
public class ConfiguratorEvent
{
	/** @modelguid {8EA8A574-06D7-480A-A4FA-B8F47A06FDDC} */
	private Object source = null;


	/** @modelguid {88692311-CC4B-47C0-8312-5EB77CB9BC58} */
	public ConfiguratorEvent(Object source)
	{
		this.source = source;
	}


	/** @modelguid {DDC37DD4-91B9-49B5-B3B7-C82CC8DAE1F1} */
	public Object getSource()
	{
		return source;
	}
}
