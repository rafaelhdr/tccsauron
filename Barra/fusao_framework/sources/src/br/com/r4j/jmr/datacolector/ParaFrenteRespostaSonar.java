package br.com.r4j.jmr.datacolector;

import java.io.IOException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import robots.Robot;
import robots.RobotHandler;
import robots.Sonar;
import robots.SonarReading;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.WebStartConfigurator;



/**
  */
public class ParaFrenteRespostaSonar
{
	private static Log logResp = LogFactory.getLog("ParaFrenteRespostaSonar");
	private static Log log = LogFactory.getLog("br.com.r4j.jmr.debug");


	private Robot myRobot = null;


	public static void main(String [] args)
	{
		try
		{
			if (args.length > 0)
				WebStartConfigurator.createConfigurator(args[0]);
			else
				WebStartConfigurator.createConfigurator("clt/conf/conf.xml");
		}
		catch (ConfiguratorException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}

		ParaFrenteRespostaSonar rTest1 = new ParaFrenteRespostaSonar();
		rTest1.run();
	}


	public void run()
	{
		try
		{
			myRobot = RobotHandler.connectToRobot("192.168.1.100", 6688, RobotHandler.iSAPHIRA_SIM_LOCAL, "test_pion1m.p");
			log.debug("myRobot = " + myRobot.getClass().getName());
			for (int i = 0; i < 8; i++)
			{
				Sonar sonar = myRobot.getSonar(i);
				if (sonar != null)
				{
					log.debug("s"+i+"_desc:(" + (int)sonar.getPosition2D().dX + "," + (int)sonar.getPosition2D().dY + "," + (int)(sonar.getPosition2D().dTh) + ")");
				}
			}
			long [] arrayLastTime = new long[8];
			for (int i = 0; i < 8; i++)
				arrayLastTime[i] = System.currentTimeMillis();

			myRobot.setVelocity(100);

			long lastReading = 0;
			long tLast = System.currentTimeMillis();
			long tFirst = tLast;
			do
			{
				long t = System.currentTimeMillis();
				long dt = t - tFirst;
				tLast = t;
				long actualReading = lastReading;
				logResp.debug("r:(" + (int)myRobot.getState().pos.dX + "," + (int)myRobot.getState().pos.dY + "," + (int)(180*myRobot.getState().pos.dTh/Math.PI) + "):dt:" + dt);
				for (int i = 0; i < 8; i++)
				{
					Sonar sonar = myRobot.getSonar(i);
					if (sonar != null && sonar.getLastReading() != null)
					{
						SonarReading sonarReading = sonar.getLastReading();
						if (sonarReading.iIdent > lastReading)
						{
							dt = t - tFirst;
							arrayLastTime[i] = t;
							if (sonarReading.iIdent > actualReading)
								actualReading = sonarReading.iIdent;
							if (sonarReading.pos != null)
							{
////								logResp.debug("s:" + i + ":i:" + sonarReading.iIdent + ":r:" + sonarReading.dRange + ":(" + (int)sonarReading.pos.dX + "," + (int)sonarReading.pos.dY  + "," + (int)(180*sonarReading.pos.dTh/Math.PI) + "):dt:" + dt);
								logResp.debug("s:" + i + ":r:" + (int)sonarReading.dRange + ":(" + (int)sonarReading.pos.dX + "," + (int)sonarReading.pos.dY  + "," + (int)(180*sonarReading.pos.dTh/Math.PI) + "):dt:" + dt);
							}
						}
					}
				}
				lastReading = actualReading;
				Thread.currentThread().sleep(20);
			}
			while (myRobot.getState().dLVel != 0 || lastReading < 100);
			RobotHandler.disconnectFromRobot("192.168.1.100", 6688, myRobot);
		}
		catch (Exception e)
		{
			RobotHandler.disconnectFromRobot("192.168.1.100", 6688, myRobot);
			e.printStackTrace();
		}
	}
}
