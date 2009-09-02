package br.com.r4j.robosim.estimator.test;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;



/**
  model.params     = zeros(model.paramdim,1);
  model.A  = [model.params(1) model.params(2); 1 0];
  model.B  = [];
  model.C  = [1 0];
  model.D  = [];
  model.G  = [1 0]';
  model.H  = [1];


  model = setparams(model,[1.9223 -0.9604]);   % 2nd order under-damped LTI system

function model = setparams(model, params, index_vector)

  if (nargin==2)
    model.params = params(:);
  elseif (nargin==3)
    model.params(index_vector) = params(:);
  else
    error('[ setparams ] Incorrect number of input arguments.');
  end

  model.A(1,:)  = model.params';


function new_state = ffun(model, state, V, U1)

  new_state      = model.A * state;

  if ~isempty(V)
      new_state(1,:) = new_state(1,:) + V(1,:);
  end


function observ = hfun(model, state, N, U2)

  observ = state(1,:);

  if ~isempty(N)
    observ = state(1,:) + N(1,:);
  end




function out = linearize(model, state, V, N, U1, U2, term, index_vector)

  if (nargin<7)
    error('[ linearize ] Not enough input arguments!');
  end

  %--------------------------------------------------------------------------------------
  switch (term)

    case 'A'
      %%%========================================================
      %%%             Calculate A = df/dstate
      %%%========================================================
      out = model.A;

    case 'B'
      %%%========================================================
      %%%             Calculate B = df/dU1
      %%%========================================================
      out = model.B;

    case 'C'
      %%%========================================================
      %%%             Calculate C = dh/dx
      %%%========================================================
      out = model.C;

    case 'D'
      %%%========================================================
      %%%             Calculate D = dh/dU2
      %%%========================================================
      out = model.D;

    case 'G'
      %%%========================================================
      %%%             Calculate G = df/dv
      %%%========================================================
      out = model.G;

    case 'H'
      %%%========================================================
      %%%             Calculate H = dh/dn
      %%%========================================================
      out = model.H;

    case 'JFW'
      %%%========================================================
      %%%             Calculate  = dffun/dparameters
      %%%========================================================
      out = [state(1) state(2); 0 0];


    case 'JHW'
      %%%========================================================
      %%%             Calculate  = dhfun/dparameters
      %%%========================================================
      out = zeros(model.obsdim,model.paramdim);

    otherwise
      error('[ linearize ] Invalid model term requested!');

  end

  if (nargin==8), out = out(:,index_vector); end

*/
public class HSensor implements Sensor, Configurable
{
	private static Log log = LogFactory.getLog(HSensor.class.getName());

	
	private String strReadingsFilePath = null;
	private double [] readings = null;
	private int idxCurrent = 0;


	public HSensor()
	{
	}


	public String getName()
	{
		return "HSensor";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		strReadingsFilePath = props.getStringProperty(strBaseKey, "readings");
	}



	public void loadData() throws IOException
	{
		LineNumberReader reader = new LineNumberReader(new FileReader(new File(strReadingsFilePath)));
		ArrayList listLines = new ArrayList();
		for (String strLine = reader.readLine(); strLine != null; strLine = reader.readLine())
			listLines.add(strLine);

		readings = new double[listLines.size()];
		for (int i = 0; i < readings.length; i++)
			readings[i] = Double.parseDouble((String) listLines.get(i));

		idxCurrent = -1;
	}


	public int getReadingsCount()
	{
		return readings.length;
	}
	
	
	public void resetData()
	{
		idxCurrent = -1;
	}


	public void dataAvailable()
	{
		idxCurrent++;
	}


	public int getDataDimension(BaseModel baseModel)
	{
		return 1;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		output.setComponent(0, readings[idxCurrent]);
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		return true;
	}



	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel baseModel)
	{
		AbstractDoubleSquareMatrix covarDist = new DoubleSquareMatrix(1);
		covarDist.setElement(0, 0, 0.3);
		return covarDist;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.Sensor#reset()
	 */
	public void reset()
	{
	}
}

