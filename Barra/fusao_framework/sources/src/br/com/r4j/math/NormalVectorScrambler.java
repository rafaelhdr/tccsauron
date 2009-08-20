package br.com.r4j.math;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;


public class NormalVectorScrambler implements VectorScrambler
{
	private static Log log = LogFactory.getLog(NormalVectorScrambler.class.getName());

	private Random rnd = null;
	private AbstractDoubleMatrix covar = null; 
	private AbstractDoubleVector mean = null; 


	public NormalVectorScrambler()
	{
		rnd = new Random((new Date()).getTime());
	}


	public void setParameters(AbstractDoubleVector mean, AbstractDoubleMatrix covar)
	{
		this.covar = covar;
		this.mean = mean;
	}


	public AbstractDoubleVector scramble(AbstractDoubleVector vectOk)
	{
		AbstractDoubleVector vectNew = new DoubleVector(vectOk.dimension());
		for (int i = 0; i < vectOk.dimension(); i++)
			vectNew.setComponent(i , vectOk.getComponent(i) + mean.getComponent(i) + Math.sqrt(covar.getElement(i, i))*(rnd.nextGaussian()));
		return vectNew;
	}
}

