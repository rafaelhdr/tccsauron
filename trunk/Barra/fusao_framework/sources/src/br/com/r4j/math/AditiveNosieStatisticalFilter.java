package br.com.r4j.math;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface AditiveNosieStatisticalFilter
{
	public void setMeasure(AbstractDoubleVector stateMeasures, AbstractDoubleSquareMatrix stateCovarMeasures);

	public void setLastState(AbstractDoubleVector stateLast, AbstractDoubleSquareMatrix stateCovarLast, AbstractDoubleSquareMatrix stateRobotCovarInc);

	public void setStateTransitionErrorInc(AbstractDoubleSquareMatrix stateRobotCovarInc);

	public void update();
	
	public AbstractDoubleVector getEstimateExpectancy();

	public AbstractDoubleSquareMatrix getEstimateCovariance();
}
