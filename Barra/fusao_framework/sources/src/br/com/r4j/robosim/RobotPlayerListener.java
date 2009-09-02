package br.com.r4j.robosim;

import java.util.EventListener;


/** @modelguid {1EFD8E49-A53F-4C0D-A1C5-F2FF392C3E40} */
public interface RobotPlayerListener extends EventListener
{
	/** @modelguid {A7EDC75D-0E83-4B6E-AB7B-5815D60A7917} */
	public void actionStarted(RobotPlayerEvent e);


	/** @modelguid {B439366B-47E6-485E-A0A9-E765B15E4477} */
	public void actionCompleted(RobotPlayerEvent e);


	/** @modelguid {3FE66066-91BD-4317-B183-C84BC5E590B8} */
	public void endOfActions(RobotPlayerEvent e);


	/** @modelguid {3958422C-A5B8-4D09-BECC-2D757A07E074} */
	public void beginOfActions(RobotPlayerEvent e);


	/**
	 * É chamado cada vez que os dados mudam.
	 * @modelguid {8F0C4E99-2D3F-4863-90B0-E579B0DCB425}
	 */
	public void actionsUpdated(RobotPlayerEvent e);
}
