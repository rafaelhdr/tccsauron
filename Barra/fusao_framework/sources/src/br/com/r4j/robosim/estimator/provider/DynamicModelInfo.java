package br.com.r4j.robosim.estimator.provider;


/** @modelguid {F3B54FAE-E7B8-4F13-AA4D-6F458AECA3CD} */
public class DynamicModelInfo
{
	/** @modelguid {5CA48CD0-A6A6-4C7A-8CEE-60EC6D6A4E06} */
	protected SensorInfo sens = null;
	/** @modelguid {531957A9-84DA-40F1-947D-906688470F2C} */
	protected String name = null;


	/** @modelguid {F563B7C2-D240-43E2-B66D-4DC0FA23F2E6} */
	public DynamicModelInfo(String name)
	{
		this.name = name;
	}


	/** @modelguid {98E010A8-B2C9-4B78-954F-B338DE6B3586} */
	public void setSensorInfo(SensorInfo sens)
	{
		this.sens = sens;
	}

	
	/** @modelguid {50D86CF3-F569-485C-9CA8-F714DF8698B8} */
	public SensorInfo getSensorInfo()
	{
		return sens;
	}


	/** @modelguid {2021F104-CDEF-46E7-A142-DF1E57C4F93D} */
	public String getName()
	{
		return name;
	}

	
	/** @modelguid {E197C2EA-A171-4751-95EF-46800288B63E} */
	public String toString()
	{
		return name;
	}


	/** @modelguid {F0F9E284-1F7F-4C73-AA67-E1D2D826EE6F} */
	public DynamicModelInfo getCopy()
	{
		DynamicModelInfo snsModelCopy = new DynamicModelInfo(name);
		 
		snsModelCopy.sens = sens;

		return snsModelCopy;
	}
}
