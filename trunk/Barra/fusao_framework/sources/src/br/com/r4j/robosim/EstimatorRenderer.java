package br.com.r4j.robosim;



public interface EstimatorRenderer
{
	public void newPose(Pose2D pose);

	public void setEstimatorRendererInfo(EstimatorRendererInfo info);


	public void setStep(int currentStep);
}
