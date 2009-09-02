package br.com.r4j.robosim;



/** @modelguid {822A6B73-A6B6-4F2A-A3AF-0EE0E16D8322} */
public interface EstimatorRenderer
{
	/** @modelguid {771EBC88-106D-4EB4-90C8-6818B58EAEE9} */
	public void newPose(Pose2D pose);

	/** @modelguid {1D2E7B98-C125-4D60-AE2E-D5113F1C313F} */
	public void setEstimatorRendererInfo(EstimatorRendererInfo info);


	/** @modelguid {A0EFC070-631B-49C7-9BC4-462222333E56} */
	public void setStep(int currentStep);
}
