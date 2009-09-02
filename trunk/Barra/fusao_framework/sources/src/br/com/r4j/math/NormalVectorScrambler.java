package br.com.r4j.math;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;


/** @modelguid {6C310C76-9CB4-474A-A2B5-E5B1FE2FA990} */
public class NormalVectorScrambler implements VectorScrambler
{
	/** @modelguid {02D6D413-7A40-4696-BD14-FA667678875E} */
	private static Log log = LogFactory.getLog(NormalVectorScrambler.class.getName());

	/** @modelguid {752D61A0-F705-4618-B779-293D210992A7} */
	private Random rnd = null;
	/** @modelguid {0C81EF81-625E-4B57-B85F-FE48E3F343D2} */
	private AbstractDoubleMatrix covar = null; 
	/** @modelguid {DD8D28AB-58D0-48DA-8DB6-BC3608FBE7DA} */
	private AbstractDoubleVector mean = null; 


	/** @modelguid {CFC86A96-A5EC-42DA-AE06-C2E2FA5BBC72} */
	public NormalVectorScrambler()
	{
		rnd = new Random((new Date()).getTime());
	}


	/** @modelguid {9408F404-248C-4D3C-BD77-583A6C32983D} */
	public void setParameters(AbstractDoubleVector mean, AbstractDoubleMatrix covar)
	{
		this.covar = covar;
		this.mean = mean;
	}


	/** @modelguid {EA95B118-51BD-4DCC-818B-F4587083F9A2} */
	public AbstractDoubleVector scramble(AbstractDoubleVector vectOk)
	{
		AbstractDoubleVector vectNew = new DoubleVector(vectOk.dimension());
		for (int i = 0; i < vectOk.dimension(); i++)
			vectNew.setComponent(i , vectOk.getComponent(i) + mean.getComponent(i) + Math.sqrt(covar.getElement(i, i))*(rnd.nextGaussian()));
		return vectNew;
	}
}

