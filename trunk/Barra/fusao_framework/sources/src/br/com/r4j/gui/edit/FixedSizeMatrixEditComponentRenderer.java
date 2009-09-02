package br.com.r4j.gui.edit;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.util.HashMap;
import java.util.Iterator;

import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.message.MessageManager;
import br.com.r4j.gui.AbstractComponentRenderer;
import br.com.r4j.gui.ComponentRenderer;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.renderer.FieldRenderer;
import br.com.r4j.m2v.view.converter.ConversionException;


/**
 * Reponsável por organizar o Layout de um EditComponent.
 * @modelguid {8BE5D674-BE9C-4155-A4F0-BEC6177B2CA1}
 */
public class FixedSizeMatrixEditComponentRenderer extends AbstractComponentRenderer implements EditComponentRenderer
{
	/** @modelguid {052D28B7-03AA-4EC4-993B-0D91A62F82D0} */
	private static Log log = LogFactory.getLog(FixedSizeMatrixEditComponentRenderer.class.getName());

	/** @modelguid {06E435C8-75C6-4CAD-9E3D-E0EEDF442130} */
	private String strUpdateLabel = "update";
	/** @modelguid {959051BD-54D4-452B-98D8-5ED7557B7E77} */
	private EditComponentModel editModel = null;
	/** @modelguid {B198CA78-52C8-40E4-8F01-4EDAE7553F5E} */
	private HashMap mapId2ErrorLabel = new HashMap();


	/** @modelguid {1D727130-01C9-4582-96F7-8BA2993437C9} */
	public FixedSizeMatrixEditComponentRenderer(EditComponentModel editModel)
	{
		super();
		this.setComponentModel(editModel);
		editModel.setRenderer(this);
		this.editModel = editModel;
	}


	/** @modelguid {944A8137-398F-48B0-B854-CE206DE5BBEC} */
	public void setUpdateLabel(String strlabel)
	{
		this.strUpdateLabel = strlabel;
	}


	/** @modelguid {CAF23E12-76D6-4A42-A20D-0E68F51AFBA1} */
	public void setErrorMessage(String strId, String strErrorMessage)
	{
		((JLabel) mapId2ErrorLabel.get(strId)).setText(strErrorMessage);
	}


	/** @modelguid {D9CFCBA6-F244-4C0B-9722-0B80595559EE} */
	protected void build()
	{
		if (contentPane == null || !editModel.hasChanged())
			return;

		this.clearFocusableComponents();
		int columns = 4;
		contentPane.removeAll();
		if (getStyleManager() != null)
			getStyleManager().prepareComponent(contentPane, GUIStyleManager.REGULAR);

		JLabel lblTitle = new JLabel();
		lblTitle.setText(this.getTitle());
		if (getStyleManager() != null)
			getStyleManager().prepareLabel(lblTitle, GUIStyleManager.TITLE);
	
		GridBagLayout gb = null; GridBagConstraints gbc = null;
		gb = new GridBagLayout(); gbc = new GridBagConstraints();
		contentPane.setLayout(gb);

		gbc.weighty = 1; gbc.fill = GridBagConstraints.HORIZONTAL;
		gbc.insets = new Insets(4, 4, 4, 4);

		gbc.anchor = GridBagConstraints.CENTER; 
		gbc.gridwidth = columns;
		gb.setConstraints(lblTitle, gbc); contentPane.add(lblTitle);

		gbc.gridwidth = 1;
		int rowCount = 1;

		Iterator itListIds = editModel.getIds().iterator();
		while (itListIds.hasNext())
		{
			String strId = (String) itListIds.next();
			String strLabel = editModel.getFieldLabel(strId);
			String strComment = editModel.getFieldComment(strId);
			FieldRenderer fieldRndr = editModel.getFieldRenderer(strId);

			JLabel lblLabel = new JLabel();
			if (getStyleManager() != null)
				getStyleManager().prepareLabel(lblLabel, GUIStyleManager.REGULAR);
			lblLabel.setText(strLabel);

			JComponent compValue = null;
			try
			{
				compValue = fieldRndr.getContentPane();
				this.addFocusableComponents(fieldRndr.getFocusableComponents());
			}
			catch (ConversionException e)
			{
				log.error("buildDialog:error", e);
				compValue = new JLabel();
				((JLabel) compValue).setText(MessageManager.getInstance().getMessage(e));
				((JLabel) compValue).setForeground(Color.RED);
			}
			if (getStyleManager() != null)
				fieldRndr.setStyle(getStyleManager(), GUIStyleManager.REGULAR);

			JLabel lblComment = new JLabel();
			if (getStyleManager() != null)
				getStyleManager().prepareLabel(lblComment, GUIStyleManager.REGULAR);
			lblComment.setText(strComment);

			JLabel lblError = new JLabel();
			lblError.setText("");
			if (getStyleManager() != null)
				getStyleManager().prepareLabel(lblError, GUIStyleManager.ERROR);

			gbc.anchor = GridBagConstraints.NORTHEAST; 
			gbc.gridx = 0;gbc.gridy = rowCount;
			gbc.weightx = 0.25;
			gbc.fill = GridBagConstraints.HORIZONTAL;
			gb.setConstraints(lblLabel, gbc); contentPane.add(lblLabel);

			gbc.anchor = GridBagConstraints.NORTHWEST;
			gbc.fill = GridBagConstraints.HORIZONTAL;
			gbc.gridx = 1;gbc.gridy = rowCount;
			gbc.weightx = 0.75;
			gb.setConstraints(compValue, gbc); contentPane.add(compValue);

			gbc.anchor = GridBagConstraints.NORTHWEST;
			gbc.gridx = 2;gbc.gridy = rowCount;
			gbc.weightx = 0;
			gbc.fill = GridBagConstraints.NONE;
			gb.setConstraints(lblError, gbc); contentPane.add(lblError);

			gbc.anchor = GridBagConstraints.NORTHEAST;
			gbc.gridx = 3;gbc.gridy = rowCount;
			gbc.weightx = 0;
			gbc.fill = GridBagConstraints.VERTICAL;
			gb.setConstraints(lblComment, gbc); contentPane.add(lblComment);

			mapId2ErrorLabel.put(strId, lblError);

			rowCount++;
		}
		JPanel pnlButtons = new JPanel();
		if (getStyleManager() != null)
			getStyleManager().prepareComponent(pnlButtons, GUIStyleManager.REGULAR);
		if (editModel.isOkEnabled())
		{
			Action actTmp = editModel.getOKAction();
			actTmp.putValue(Action.NAME, this.getOkLabel());
			JButton btnOk = new JButton(actTmp);
			this.addFocusableComponent(btnOk);
			if (getStyleManager() != null)
				getStyleManager().prepareButton(btnOk, GUIStyleManager.REGULAR);
			pnlButtons.add(btnOk);
			contentPane.registerKeyboardAction(editModel.getOKAction(), ComponentRenderer.strokeDefaultOk, JComponent.WHEN_IN_FOCUSED_WINDOW);
		}
		if (editModel.isUpdatableEnabled())
		{
			Action actTmp = editModel.getUpdateAction();
			actTmp.putValue(Action.NAME, strUpdateLabel);
			JButton btnUpdt = new JButton(actTmp);
			this.addFocusableComponent(btnUpdt);
			if (getStyleManager() != null)
				getStyleManager().prepareButton(btnUpdt, GUIStyleManager.REGULAR);
			pnlButtons.add(btnUpdt);
		}
		if (editModel.isCancelEnabled())
		{
			Action actTmp = editModel.getCancelAction();
			actTmp.putValue(Action.NAME, this.getCancelLabel());
			JButton btnCancel = new JButton(actTmp);
			this.addFocusableComponent(btnCancel);
			if (getStyleManager() != null)
				getStyleManager().prepareButton(btnCancel, GUIStyleManager.REGULAR);
			pnlButtons.add(btnCancel);
		}
		gbc.anchor = GridBagConstraints.WEST; 
		gbc.gridx = 0;gbc.gridy = rowCount;
		gbc.weightx = (float) 1;
		gbc.gridwidth = columns;
		gbc.fill = GridBagConstraints.NONE;
		gb.setConstraints(pnlButtons, gbc); contentPane.add(pnlButtons);
	}
}

