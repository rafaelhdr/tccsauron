package br.com.r4j.robosim.gui.actions;

import java.awt.event.ActionEvent;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.Icon;
import javax.swing.JOptionPane;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.FileChooserWindow;
import br.com.r4j.gui.GenericFilters;
import br.com.r4j.gui.ImagePreview;


/**
 *
 * @modelguid {F279F157-BA42-4305-B42B-267A35960FAD}
 */
public class FileAction extends AbstractAction
{
	/** @modelguid {DED09C67-6F84-4C85-8B07-E63D4BFB7268} */
	private static Log log = LogFactory.getLog(FileAction.class.getName());

	/** @modelguid {E1FAAB33-335E-4301-ADA6-77F528446869} */
	public static final int IMAGE_LIST = 1;

	/** @modelguid {640A54FF-9BEE-4FDF-9A38-13D4CDA31933} */
	private List listFilter = null;
	/** @modelguid {8872E08E-3099-4B8A-BDEA-BC66D55C724A} */
	private List listFilterDesc = null;
	/** @modelguid {9545D9AC-CC65-4C34-9C54-47D1FE59409E} */
	private int accId = -1;
	/** @modelguid {CF8670FF-C92A-491B-9FFA-8115DEBBCCE5} */
	private String strPath = null;
	/** @modelguid {72D64451-C933-4028-80DE-E33E9D108331} */
	private Object objCallback = null;
	/** @modelguid {FD4970BD-3350-4C9D-A028-2F0C43396E73} */
	private String strMethodName = null;


	/** @modelguid {88B6308D-AB41-45B4-A277-8B0391FC3B69} */
	public FileAction(Icon icon, String strDescription, Object objCallback, String strMethodName)
	{
		super(null, icon);
		this.putValue(Action.SHORT_DESCRIPTION, strDescription);
		listFilter = new ArrayList();
		listFilterDesc = new ArrayList();
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	/** @modelguid {634BA058-7AD4-49FE-9007-0BDAF3C34B03} */
	public FileAction(String strName, Object objCallback, String strMethodName)
	{
		super(strName);
		listFilter = new ArrayList();
		listFilterDesc = new ArrayList();
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	/** @modelguid {DF04EC2B-F0E1-471F-8E57-8AA48B399C9F} */
	public FileAction(String strName, String strButtonLabel, List listFilter, List listFilterDesc, Object objCallback, String strMethodName)
	{
		super(strName);
		this.listFilter = listFilter;
		this.listFilterDesc = listFilterDesc;
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	/** @modelguid {54165836-F5D1-4F22-8C4C-58B2BFAFF391} */
	public void setAccessory(int accId)
	{
		this.accId = accId;
	}


	/** @modelguid {BCF48501-6EBE-405E-B809-D9BE8B45F44B} */
	public void actionPerformed(ActionEvent e)
	{
		try
		{
			FileChooserWindow fcwPathGenerator = new FileChooserWindow("Abrir", null, true);

			Iterator itFilters = listFilter.iterator();
			Iterator itFilterDescs = listFilterDesc.iterator();
			while (itFilters.hasNext())
			{
				String strFilter = (String) itFilters.next();
				String strFilterName = (String) itFilterDescs.next();

				GenericFilters ffRet = new GenericFilters(strFilter, strFilterName);
				ffRet.setExtensionListInDescription(true);
				fcwPathGenerator.addChoosableFileFilter(ffRet);
			}

			if (Actions.currentDir != null)
				fcwPathGenerator.setCurrentDirectory(Actions.currentDir);

			switch (accId)
			{
				case IMAGE_LIST:
				{
					fcwPathGenerator.setAccessory(new ImagePreview(fcwPathGenerator));
					break;
				}
				default:
				{
					break;
				}
			}
			Actions.currentDir = fcwPathGenerator.getCurrentDirectory();
			strPath = fcwPathGenerator.getFileName();

			Class clsCallback = objCallback.getClass();
			Class [] arrayClasses = new Class[1]; arrayClasses[0] = String.class;
			Object [] arrayValues = new Object[1]; arrayValues[0] = strPath;
			Method meth = clsCallback.getMethod(strMethodName, arrayClasses);
			meth.invoke(objCallback, arrayValues);
		}
		catch (Exception exc)
		{
			log.debug("FileAction:actionPerformed:error", exc);
			JOptionPane.showMessageDialog(null, "Não foi possível abrir a imagem.");
		}
	}


	/** @modelguid {69B877A1-5A5F-4E9A-A645-EEEB030C95BC} */
	public String getPath()
	{
		return strPath;
	}
}
