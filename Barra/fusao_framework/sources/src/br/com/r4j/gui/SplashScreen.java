package br.com.r4j.gui;


import java.awt.BorderLayout;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Image;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JWindow;

/**
 * @modelguid {50836EBA-F5E2-4912-90A8-E7B19537E098}
*/
public class SplashScreen extends JWindow
{
	/** @modelguid {6A8F0BA9-702C-4E76-9164-E5900332F1DA} */
   private static final int   m_iconsHeight = 30;
	/** @modelguid {9683610E-920C-42E2-906A-93B288FD4ADC} */
   private static final int   m_iconsWidth = 30;

	/** @modelguid {3B488237-07FD-41BD-9365-D53085CB7761} */
   private int                m_splashWidth;
	/** @modelguid {6D4AEAB8-FD3A-49F6-9DE4-FA214FBFA4A9} */
   private int                m_splashHeight;
	/** @modelguid {3CBF88F7-2F3B-480E-9990-870BE7F91865} */
   private ImageIcon          m_icon;                                   // splash image
	/** @modelguid {0745558B-3B07-4691-825C-5184ECE9E5FE} */
   private ImageIcon[]        m_icons = new ImageIcon[100];             // product icons
	/** @modelguid {AB27C0CA-EB95-4352-9165-DD7E806AF89B} */
   private JLabel             m_splashLabel;
	/** @modelguid {C3AB1D81-E7BF-400C-A734-FF6EB2046222} */
   private JPanel             m_iconsPanel;
	/** @modelguid {A5B0C4BB-FE22-4475-8F06-155C636C8A9B} */
   private boolean            m_bKilled = false;
	/** @modelguid {5AFA816C-61DB-4B97-9670-BDF319D53540} */
   private int                m_iconsIdx = 0;                           // number of product icons
	/** @modelguid {ED2EE8A1-4AE0-47CE-96CE-7E3390F0B655} */
   private boolean            m_bLayout = false;
	/** @modelguid {D78AFF36-63FA-4AF6-9985-7854F8E3E01F} */
   private boolean            m_bWaiting = false;
   

	/** @modelguid {6F334D3D-B224-4B34-B4DF-B227A511D87D} */
	public SplashScreen(Frame frame, Image image, final long tmWaiting)
	{
		this(frame, new ImageIcon(image));
		
		Thread th = new Thread(new Runnable()
		{
			public void run()
			{
				try
				{
					Thread.sleep(tmWaiting);
				}
				catch (Exception e) {}
				kill();				
			}
		}
		);
		th.start();
	}


	/** @modelguid {0F2255F9-A0E0-44E5-B212-92E454D38D8E} */
	public SplashScreen(Frame frame, Image image)
	{
		this(frame, new ImageIcon(image));
	}


	/**
	 * @modelguid {C2237FAB-9ABB-4779-A9F6-A495F3713162}
	*/
	public SplashScreen(Frame frame, ImageIcon image)
	{
		super((null == frame) ? new Frame() : frame);
		try
		{
			this.getContentPane().setLayout(new BorderLayout());
			this.changeSplashImage(image);
		}
		catch (Exception e)
		{
			System.out.println("Exception in SplashScreen constructor - " + e);
			// don't try to do anything else if there was a problem.
			return;
		}
		catch(Error e)
		{
			System.out.println("Error in SplashScreen constructor - " + e);
			// don't try to do anything else if there was a problem.
			return;
		}
/*
		// interrupt the thread if the mouse is clicked.
		MouseAdapter ma = new MouseAdapter()
		{
			public void mousePressed(MouseEvent me)
			{
				kill();
			}
		};

		//addMouseListener( ma );

		addKeyListener(new KeyAdapter()
		{
			public void keyPressed(KeyEvent e)
			{
				if(e.getKeyCode() == e.VK_SPACE)
					kill();
			}
		});
//*/

		// change the cursor to the wait cursor
		this.startWaitCursor();
	}


	/** @modelguid {30354FDC-E2F5-45CD-ADA9-EB2B00D5499C} */
	public void setVisible(final boolean visible)
	{
		superSetVisible(visible);
	}


	/** @modelguid {80604E61-C057-4A93-81B1-F3BD8C2C92B6} */
	private void superSetVisible(boolean visible)
	{
		super.setVisible(visible);
	}


	// Sets the dialog cursor to a wait cursor.
	/** @modelguid {EC65B8F6-B737-4C95-840D-4428D696AF9E} */
	private void startWaitCursor()
	{
		if(!m_bWaiting)
		{
			setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
			m_bWaiting = true;
		}
	}


	// Sets the cursor back to normal from a wait cursor.
	/** @modelguid {D952749B-CA3E-4ED2-8775-EB8EAD77F332} */
	private void endWaitCursor()
	{
		if(m_bWaiting)
		{
			m_bWaiting = false;
			setCursor(Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR));
		}
	}


	/**
	 * Set the splash screen background, icons panel and redisplay window.
	 * @modelguid {DD43A0B1-5667-40DD-911A-47C9D2950F45}
	 */
	synchronized public void changeSplashImage(final ImageIcon icon)
	{
		this.m_icon = icon;
		this.m_splashHeight = m_icon.getIconHeight();
		this.m_splashWidth = m_icon.getIconWidth();

		// set the window size based on the size of splash image
		this.setSize(m_splashWidth, m_splashHeight + m_iconsHeight + 12);

		// define splashlabel layout
		this.m_splashLabel = new JLabel(m_icon)
		{
			public void paintComponent(Graphics g)
			{
				super.paintComponent(g);
				int tleft = 76;
				int ttop = 133;
				Font f = g.getFont();
				Font newFont = new Font(f.getName(),f.getStyle(),f.getSize()-2);
				g.setFont(newFont);
				// g.drawString(Version.getDisplayVersion(), this.getInsets().left+tleft, this.getInsets().top+ttop); 
			}
		};
		this.displaySplash();
	}


	/**
	* Add text to the bottom of the splash screen.
	* @modelguid {E3E730BB-A6D3-4D9D-BCA1-BBC7006FAC2E}
	*/
	synchronized public void addText(final String msg )
	{
		m_iconsPanel.removeAll();

		// clear and setup the icon panel
		JLabel txtLabel = new JLabel(msg);//COMMENTED BY VINOD V

		Dimension d = txtLabel.getPreferredSize();
		d.height = m_iconsHeight;
		txtLabel.setPreferredSize(d);
		m_iconsPanel.add(txtLabel);

		// redisplay icon panel on splash window
		m_iconsPanel.doLayout();
		m_iconsPanel.repaint();
	}


	/**
	 * Add an icon to the bottom of the splash screen.
	 * @modelguid {BDD99B24-C8D8-46BE-9A40-3B1CC209FB37}
	 */
	synchronized public void addIcon(final ImageIcon icon )
	{
		int numIcons = m_splashWidth / m_iconsWidth;
		if (m_iconsIdx < numIcons)
		{
			m_icons[m_iconsIdx] = icon;
			m_iconsIdx++;
		}
		else
		{
			// move everything down one and then insert it
			for (int i=0; i<numIcons-1; i++)
				m_icons[i] = m_icons[i+1];
			m_icons[numIcons-1] = icon;
			m_bLayout = true;
		}
	}


	/**
	* Setup the window layout and display it.
	* @modelguid {1C84B43E-D079-465F-A428-C192338F8E84}
	*/
	synchronized private void displaySplash()
	{
		System.err.println("displaySplash()");
		this.getContentPane().removeAll();      // remove all containers from window

		// add mainLabel to window
		this.getContentPane().add(m_splashLabel, BorderLayout.CENTER);
		this.pack();
		this.setSize(m_splashWidth, m_splashHeight + m_iconsHeight + 12);

		// Center the dialog over the window
		GUIFactory.centerWindow(this);
		this.setVisible(true);
	}


	/**
	* Kill the splash screen.
	* @modelguid {F0C0A32F-F8EC-4DDC-8530-DCC366436340}
	*/
	synchronized public void kill()
	{
		System.err.println("kill()");
		if (!m_bKilled)
		{ 
			dispose();
			m_bKilled = true;

			// set the cursor back to the default cursor
			endWaitCursor();
		}
	}
}
