package Interfaz;

import java.awt.BorderLayout;
import java.awt.EventQueue;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;
import javax.swing.JButton;
import javax.swing.JSlider;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

public class Interfaz01 extends JFrame {

	private JPanel contentPane;
	private JButton run;
	private JButton stop;
	private JSlider slider;
	private JSlider slider_1;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Interfaz01 frame = new Interfaz01();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the frame.
	 */
	public Interfaz01() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 450, 300);
		contentPane = new JPanel();
		contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		setContentPane(contentPane);
		contentPane.setLayout(null);
		{
			run = new JButton("Run");
			run.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					
					
				}
			});
			run.setBounds(301, 173, 85, 21);
			contentPane.add(run);
		}
		
		//funcion de csdynmatlab
		//public static int csdynmatlab([float t,float xinit]){
			
			
		//}
		
		{
			stop = new JButton("Stop");
			stop.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					
					
				}
			});
			stop.setBounds(301, 204, 85, 21);
			contentPane.add(stop);
		}
		{
			slider = new JSlider();
			slider.setBounds(35, 173, 200, 22);
			contentPane.add(slider);
		}
		{
			slider_1 = new JSlider();
			slider_1.setBounds(35, 204, 200, 22);
			contentPane.add(slider_1);
		}
	}
}
