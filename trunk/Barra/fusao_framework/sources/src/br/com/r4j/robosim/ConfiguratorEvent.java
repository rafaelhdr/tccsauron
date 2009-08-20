package br.com.r4j.robosim;



public class ConfiguratorEvent
{
	private Object source = null;


	public ConfiguratorEvent(Object source)
	{
		this.source = source;
	}


	public Object getSource()
	{
		return source;
	}
}
