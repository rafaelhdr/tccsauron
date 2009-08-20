package br.com.r4j.robosim;

import java.util.EventListener;


public interface RobotPlayerListener extends EventListener
{
	public void actionStarted(RobotPlayerEvent e);


	public void actionCompleted(RobotPlayerEvent e);


	public void endOfActions(RobotPlayerEvent e);


	public void beginOfActions(RobotPlayerEvent e);


	/**
	 * É chamado cada vez que os dados mudam.
	 */
	public void actionsUpdated(RobotPlayerEvent e);
}
