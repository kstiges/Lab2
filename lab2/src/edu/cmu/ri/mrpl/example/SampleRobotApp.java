package edu.cmu.ri.mrpl.example;
/*

 * Sample code for the 16x62 class taught at the Robotics Institute
 * of Carnegie Mellon University
 *
 * written from scratch by Martin Stolle in 2006
 *
 * inspired by code started by Illah Nourbakhsh and used
 * for many years.
 */

import java.io.*;
import java.text.NumberFormat;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;

import javax.swing.*;

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import static java.lang.Math.*;

public class SampleRobotApp extends JFrame implements ActionListener, TaskController {

	private Robot robot;
	private SonarConsole sc;
	
	private JFrame scFrame;

	private JButton connectButton;
	private JButton disconnectButton;

	private JButton pauseButton;
	private JButton waitButton;
	private JButton turnToButton;
	private JButton goToButton;
	private JFormattedTextField argumentField;
	private JFormattedTextField remainingField;

	private JButton stopButton;
	private JButton quitButton;

	private PauseTask pauseTask;
	private WaitTask waitTask;
	private TurnToTask turnToTask;
	private GoToTask goToTask;
	
	static final int DEFAULT_ROOM_SIZE = 4;
	
	private Task curTask = null;

	public SampleRobotApp() {
		super("Kick Ass Sample Robot App");

		connectButton = new JButton("connect");
		disconnectButton = new JButton("disconnect");

		pauseButton = new JButton("Pause!");
		waitButton = new JButton("Wait!");
		turnToButton = new JButton("Turn to angle!");
		goToButton = new JButton("Go to distance!");
		argumentField = new JFormattedTextField(NumberFormat.getInstance());
		argumentField.setValue(0);
		NumberFormat nf = NumberFormat.getInstance();
		nf.setMaximumFractionDigits(2);
		remainingField = new JFormattedTextField(nf);
		remainingField.setEditable(false);
		stopButton = new JButton(">> stop <<");
		quitButton = new JButton(">> quit <<");

		connectButton.addActionListener(this);
		disconnectButton.addActionListener(this);

		pauseButton.addActionListener(this);
		waitButton.addActionListener(this);
		turnToButton.addActionListener(this);
		goToButton.addActionListener(this);
		stopButton.addActionListener(this);
		quitButton.addActionListener(this);

		Container main = getContentPane();
		main.setLayout(new BoxLayout(main, BoxLayout.Y_AXIS));
		Box box;

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(connectButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(disconnectButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(pauseButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(waitButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(turnToButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(goToButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));
		
		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(argumentField);
		box.add(Box.createHorizontalStrut(30));
		box.add(remainingField);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(stopButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(quitButton);

		main.add(Box.createVerticalStrut(30));


		this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		this.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				quit();
			}
		});


		this.setLocationByPlatform(true);
		this.pack();
		// moved this line to the bottom to make the controls show up on top
		//this.setVisible(true);

		// construct the SonarConsole, but don't make it visible
		scFrame = new JFrame("Sonar Console");
		scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		sc = new SonarConsole();
		scFrame.add(sc);
		scFrame.pack();
		
		// construct tasks
		pauseTask = new PauseTask(this);
		waitTask = new WaitTask(this);
		turnToTask = new TurnToTask(this);
		goToTask = new GoToTask(this);
		
		this.setVisible(true);

		String message = "Is this running on the robot?";
		int isRobot = JOptionPane.showConfirmDialog(null, message, message, JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE);
		if (isRobot == JOptionPane.YES_OPTION) {
			robot = new ScoutRobot();
		}
		else {
			robot = new SimRobot();
		}
	}

	// call from GUI thread
	public void connect() {
		try {
			robot.connect();
		} catch(IOException ioex) {
			System.err.println("couldn't connect to robot:");
			ioex.printStackTrace();
		}
	}

	// call from GUI thread
	public void disconnect() {
		try {
			robot.disconnect();
		} catch(IOException ioex) {
			System.err.println("couldn't disconnect from robot:");
			ioex.printStackTrace();
		}
	}

	// call from GUI thread
	public synchronized void stop() {
		if (curTask!=null)
			curTask.pleaseStop();
	}

	// call from GUI thread
	public void quit() {

		synchronized(this) {
			if (curTask!=null) {
				curTask.pleaseStop();

				try {
					this.wait(1000);
				} catch(InterruptedException iex) {
					System.err.println("interrupted while waiting for worker termination signal");
				}
			}

			if (curTask!=null) {
				System.err.println("exiting even though robot is running!");
			} else {
				disconnect();
			}
		}

		System.exit(0);
	}

	// invoked from non-GUI thread to hide/show SonarConsole
	public void showSC() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				scFrame.setLocationByPlatform(true);
				scFrame.setVisible(true);
			}
		});
	}

	// invoked from non-GUI thread to hide/show SonarConsole
	public void hideSC() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				scFrame.setVisible(false);
			}
		});
	}

	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();
		
		double argument = Double.parseDouble(argumentField.getText());
		
		if (source==connectButton) {
			connect();
		} else if ( source==disconnectButton ) {
			disconnect();
		} else if ( source==stopButton ) {
			stop();
		} else if ( source==quitButton ) {
			quit();
		} else if ( source==pauseButton ) {
			(new Thread(pauseTask)).start();
		} else if ( source==waitButton ) {
			// read argument from text field
			waitTask.setDuration(argument);
			(new Thread(waitTask)).start();
		} else if ( source==turnToButton ) {
			turnToTask.setDesiredAngle(argument);
			(new Thread(turnToTask)).start();
		} else if ( source==goToButton ) {
			(new Thread(goToTask)).start();
		}
	}

	// TODO allow more than one task to run?
	public synchronized boolean canStart(Task t) {
		if (curTask!=null)
			return false;

		curTask = t;
		return true;
	}

	public synchronized void finished(Task t) {
		if (curTask!=t) {
			System.err.println("ignoring finished() from unknown task "+t+"!");
			return;
		}
		curTask=null;
		this.notifyAll();
	}

	public static void main(String[] argv) {
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				new SampleRobotApp();
			}
		});
	}

	// here we implement the different robot controllers
	// as Tasks
	//
	
	class PauseTask extends Task implements KeyListener, MouseListener {
		
		Speech speech;
		boolean done;

		PauseTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			// XXX this doesn't work right now
			//speech.speak("Waiting for you");
			done = false;
			long startTime = System.currentTimeMillis();

			// XXX these only catch events on the specific object
			pauseButton.addKeyListener(this);
			SampleRobotApp.this.addMouseListener(this);

			while(!shouldStop() && !done) {
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("pause sleep interrupted");
				}
			}
			
			long endTime = System.currentTimeMillis();
			double elapsedTime = (endTime - startTime) / 1000.0;
			remainingField.setValue(elapsedTime);
		}

		public String toString() {
			return "pause task";
		}

		public void keyPressed(KeyEvent e) {
			done = true;
			SampleRobotApp.this.removeKeyListener(this);
		}
		
		public void mouseClicked(MouseEvent arg0) {
			done = true;
			SampleRobotApp.this.removeMouseListener(this);
		}

		public void keyReleased(KeyEvent e) {}
		public void keyTyped(KeyEvent e) {}
		public void mouseEntered(MouseEvent arg0) {}
		public void mouseExited(MouseEvent arg0) {}
		public void mousePressed(MouseEvent arg0) {}
		public void mouseReleased(MouseEvent arg0) {}
	}
	
	class WaitTask extends Task {

		Speech speech;
		double duration;
		private long startTime;

		WaitTask(TaskController tc) {
			super(tc);
		}
		
		public void setDuration (double d) {
			duration = d;
		}

		public void taskRun() {
			//XXX this doesn't work now
			//speech = new Speech();
			sayDuration();
			startTime = System.currentTimeMillis();
			double elapsed = elapsedTime();
			
			while(!shouldStop() && elapsed < duration) {
				elapsed = elapsedTime();
				if (elapsed >= duration) {
					break;
				}
				try {
					// XXX is this the right place to update this box?
					remainingField.setValue(duration - elapsed);
					
					
					Thread.sleep(5);
				} catch(InterruptedException iex) {
					System.out.println("wait sleep interrupted");
				}
			}
			// final update
			remainingField.setText(duration - elapsed + "");
		}
		
		private void sayDuration () {
			// cut off past two significant figures
			double roundedDuration = ((int) (duration*100)) / 100.0;
			if (speech != null) {
				speech.speak("Waiting " + roundedDuration + " seconds");
			}
		}

 		private double elapsedTime() {
			return (System.currentTimeMillis() - startTime) / 1000.0;
		}

		public String toString() {
			return "wait task";
		}
	}

	class TurnToTask extends Task {
		
		double desiredAngle;
		Controller controller;

		TurnToTask(TaskController tc) {
			super(tc);
		}
		
		public void setDesiredAngle (double a) {
			desiredAngle = a;
		}

		public void taskRun() {
			controller = new Controller(robot);
			final double Kp = 0.4;
			final double Kd = 50000000;
			double u = 0;
			robot.updateState();
			double curAngle = robot.getHeading();
			double lastAngle = curAngle;
			double curTime = System.nanoTime();
			double lastTime = curTime;
			double destAngle = curAngle + desiredAngle;
			double angleErr = AngleMath.add(destAngle, -curAngle);
			
			final double ANGLE_TOLERANCE = 0.05;

			while(!shouldStop() &&
					(abs(angleErr) > ANGLE_TOLERANCE 
							|| robot.getVelLeft() != 0) )
			{
				robot.updateState();
				lastAngle = curAngle;
				lastTime = curTime;
				curAngle = robot.getHeading();
				curTime = System.nanoTime();
				angleErr = AngleMath.add(destAngle, -curAngle);
				
				double angleTraveled = AngleMath.add(curAngle, -lastAngle);
				boolean progressMade = signum(angleTraveled) == signum(angleErr);

				double pterm = Kp*angleErr;
				double dterm = Kd*(abs(angleTraveled))/(curTime-lastTime);
				if (progressMade) {
					dterm *= -1;
				}
				System.err.println(dterm);
				/*
				u = pterm;
				/*/
				u = pterm + dterm;
				//*/
				double direction = signum(angleErr);
				//System.err.println(angleErr + " off, speed = " + u*direction);
				double speed = u;
				controller.setVel(-speed, speed);
				
				try {
					Thread.sleep(5);
				} catch(InterruptedException iex) {
					System.out.println("turn-to sleep interrupted");
				}
			}
			
			// TODO speak error aloud
			controller.stop();
		}

		public String toString() {
			return "turn-to";
		}
	}
	
	private enum WallFollowState {
		GO, TURN, STOP
	}

	class GoToTask extends Task {
		
		double desiredDistance;
		RealPose2D robotStartedHere;
		Controller controller;
		Perceptor perceptor;

		GoToTask(TaskController tc) {
			super(tc);
		}
		
		public void setDesiredDistance (double d) {
			desiredDistance = d;
			robotStartedHere = perceptor.getPose();
		}

		public void taskRun() {
			controller = new Controller(robot);
			perceptor = new Perceptor(robot);
			final double Kp = 0.4;
			final double Kd = 50000000;
			double u = 0;
			RealPose2D curPose = perceptor.getPose();
			RealPose2D lastPose = curPose;
			double curTime = System.nanoTime();
			double lastTime = curTime;
			Point2D destPoint = curPose.transform(new Point2D.Double(desiredDistance, 0), null);
			double distanceErr = curPose.getPosition().distance(destPoint);
			
			final double DISTANCE_TOLERANCE = 0.05;

			while(!shouldStop() &&
					(abs(distanceErr) > DISTANCE_TOLERANCE 
							|| robot.getVelLeft() != 0) )
			{
				robot.updateState();
				lastPose = curPose;
				lastTime = curTime;
				curPose = perceptor.getPose();
				curTime = System.nanoTime();
				distanceErr = curPose.inverseTransform(destPoint, null).getX();
						
				// XXX this is always positive right now
				// this means that the dterm won't work right
				double distanceTraveled = curPose.getPosition().distance(lastPose.getPosition());
				boolean progressMade = signum(distanceTraveled) == signum(distanceErr);

				double pterm = Kp*distanceErr;
				double dterm = Kd*(abs(distanceTraveled))/(curTime-lastTime);
				if (progressMade) {
					dterm *= -1;
				}
				System.err.println(dterm);
				//*
				u = pterm;
				/*/
				u = pterm + dterm;
				//*/
				double direction = signum(distanceErr);
				//System.err.println(angleErr + " off, speed = " + u*direction);
				double speed = u;
				controller.setVel(speed, speed);
				
				try {
					Thread.sleep(5);
				} catch(InterruptedException iex) {
					System.out.println("go-to sleep interrupted");
				}
			}
			
			// TODO speak error aloud
			controller.stop();
		}

		public String toString() {
			return "go-to";
		}
	}
	
	private static final long serialVersionUID = 0;
}
