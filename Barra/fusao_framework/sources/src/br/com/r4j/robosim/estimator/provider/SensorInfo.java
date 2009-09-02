package br.com.r4j.robosim.estimator.provider;



/** @modelguid {29854827-7FAF-4D55-8394-D57650035352} */
public class SensorInfo
{
	/** @modelguid {DB7F48E6-EE40-43A6-A5A1-30ACAA90BA97} */
	private String name = null;


	/** @modelguid {F363649B-E176-41A5-97C2-89F306E38ACF} */
	public SensorInfo(String name)
	{
		this.name = name;
	}

	
	/** @modelguid {AD22DE6E-BA80-4936-955E-CE8030C1AA7F} */
	public String getName()
	{
		return name;
	}

	
	/** @modelguid {E9C213A7-F00B-44BC-B424-802CFB28167F} */
	public String toString()
	{
		return name;
	}
}
