/*
 * Created on Nov 27, 2005
 *
 * Window - Preferences - Java - Code Style - Code Templates
 */
package br.com.r4j.gui;

import javax.swing.JFrame;

/**
 * @author giord
 *
 * Window - Preferences - Java - Code Style - Code Templates
 */
public class TesteGUI extends JFrame {

	private javax.swing.JPanel jContentPane = null;

	/**
	 * This is the default constructor
	 */
	public TesteGUI() {
		super();
		initialize();
	}
	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(300,200);
		this.setContentPane(getJContentPane());
		this.setTitle("JFrame");
	}
	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private javax.swing.JPanel getJContentPane() {
		if(jContentPane == null) {
			jContentPane = new javax.swing.JPanel();
			jContentPane.setLayout(new java.awt.BorderLayout());
		}
		return jContentPane;
	}
}
