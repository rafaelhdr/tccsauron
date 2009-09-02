package br.com.r4j.math;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {22C8BC42-8E2C-4BBD-8120-D0E8868B3137} */
public interface AditiveNosieStatisticalFilter
{
	/** @modelguid {BF3EC6D6-CE1A-4B30-B4FE-76808B7EAD51} */
	public void setMeasure(AbstractDoubleVector stateMeasures, AbstractDoubleSquareMatrix stateCovarMeasures);

	/** @modelguid {6AB6D127-15C5-493D-A0DC-836FE6D1F35E} */
	public void setLastState(AbstractDoubleVector stateLast, AbstractDoubleSquareMatrix stateCovarLast, AbstractDoubleSquareMatrix stateRobotCovarInc);

	/** @modelguid {5791F5F4-8217-4000-8FA9-7CB524B58C3B} */
	public void setStateTransitionErrorInc(AbstractDoubleSquareMatrix stateRobotCovarInc);

	/** @modelguid {DC8A6391-572B-41C3-B7B9-BECF9B02CBC5} */
	public void update();
	
	/** @modelguid {63A12411-FCDC-4233-B9BF-7E8F363B3134} */
	public AbstractDoubleVector getEstimateExpectancy();

	/** @modelguid {B3EA1D9F-FEED-4FBD-94FB-21B6BBB0CCF1} */
	public AbstractDoubleSquareMatrix getEstimateCovariance();
}
