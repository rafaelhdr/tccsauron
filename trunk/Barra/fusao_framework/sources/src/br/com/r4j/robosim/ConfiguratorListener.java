package br.com.r4j.robosim;

import java.util.EventListener;


public interface ConfiguratorListener extends EventListener
{
	public void configurationDone(ConfiguratorEvent e);
}
