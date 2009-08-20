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
 */
public class FileAction extends AbstractAction
{
	private static Log log = LogFactory.getLog(FileAction.class.getName());

	public static final int IMAGE_LIST = 1;

	private List listFilter = null;
	private List listFilterDesc = null;
	private int accId = -1;
	private String strPath = null;
	private Object objCallback = null;
	private String strMethodName = null;


	public FileAction(Icon icon, String strDescription, Object objCallback, String strMethodName)
	{
		super(null, icon);
		this.putValue(Action.SHORT_DESCRIPTION, strDescription);
		listFilter = new ArrayList();
		listFilterDesc = new ArrayList();
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	public FileAction(String strName, Object objCallback, String strMethodName)
	{
		super(strName);
		listFilter = new ArrayList();
		listFilterDesc = new ArrayList();
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	public FileAction(String strName, String strButtonLabel, List listFilter, List listFilterDesc, Object objCallback, String strMethodName)
	{
		super(strName);
		this.listFilter = listFilter;
		this.listFilterDesc = listFilterDesc;
		this.objCallback = objCallback;
		this.strMethodName = strMethodName;
	}


	public void setAccessory(int accId)
	{
		this.accId = accId;
	}


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


	public String getPath()
	{
		return strPath;
	}
}
