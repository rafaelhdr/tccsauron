package br.com.r4j.robosim.estimator;

import java.util.List;

import JSci.maths.*;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.robosim.RobotTrack;
import br.com.r4j.robosim.EstimatorRenderer;


public interface ParticleFilter
{
	public AbstractDoubleMatrix getParticles();
}
