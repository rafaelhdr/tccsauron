package br.com.r4j.robosim;



/** @modelguid {1E25AB76-61CE-4F50-8149-B0212F51742A} */
public class RobotPlayerEvent
{
	/** @modelguid {42C18621-20F0-4E20-94BA-397267551398} */
	private Object source = null;
	/** @modelguid {950E9541-C4C6-4C6F-8239-F4525051BE9C} */
	private int stepIdx = -1;


	/** @modelguid {3B934015-F98B-4984-A64B-B6EBCD87F276} */
	public RobotPlayerEvent(Object source, int stepIdx)
	{
		this.source = source;
		this.stepIdx = stepIdx;
	}


	/** @modelguid {66FB2F48-CB7E-456B-97C2-DCB581BC4091} */
	public Object getSource()
	{
		return source;
	}


	/** @modelguid {6BDFE4D4-3EA3-47DA-8CA2-DB1E59E0A337} */
	public int getStepIdx()
	{
		return stepIdx;
	}
}
