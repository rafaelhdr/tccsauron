package br.com.r4j.robosim;

import java.util.EventListener;


/** @modelguid {F93584E3-E21E-4F92-97F6-AC03365B895D} */
public interface ConfiguratorListener extends EventListener
{
	/** @modelguid {A495B1FD-B333-432C-A910-D006D34FFFAE} */
	public void configurationDone(ConfiguratorEvent e);
}
