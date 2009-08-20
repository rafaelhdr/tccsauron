package br.com.r4j.robosim;



public class RobotPlayerEvent
{
	private Object source = null;
	private int stepIdx = -1;


	public RobotPlayerEvent(Object source, int stepIdx)
	{
		this.source = source;
		this.stepIdx = stepIdx;
	}


	public Object getSource()
	{
		return source;
	}


	public int getStepIdx()
	{
		return stepIdx;
	}
}
