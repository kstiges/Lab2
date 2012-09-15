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
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import static edu.cmu.ri.mrpl.RobotModel.*;
import edu.cmu.ri.mrpl.util.*;
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
	private JTextField remainingField;

	private JButton stopButton;
	private JButton quitButton;

	private PauseTask pauseTask;
	private WaitTask waitTask;
	private TurnToTask turnToTask;
	private GoToTask goToTask;
	
	private PointsConsole pc;
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
		remainingField = new JTextField();
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
		
		int pcDim = 480;
		pc = new PointsConsole(768-pcDim, 1024-pcDim, pcDim, pcDim);
		pc.setWorldViewport(-1.0 * DEFAULT_ROOM_SIZE, -1.0 * DEFAULT_ROOM_SIZE, 1.0 * DEFAULT_ROOM_SIZE - 0.5, 1.0 * DEFAULT_ROOM_SIZE - 0.5);

		// construct tasks
		pauseTask = new PauseTask(this);
		waitTask = new WaitTask(this);
		turnToTask = new TurnToTask(this);
		goToTask = new GoToTask(this);
		
		Frame f = new Frame();
		f.setTitle("Points Console");
		
		f.add(pc.getPanel());
		
		f.setSize(pc.getPanel().getSize());
		f.setVisible(true);
		//f.setLocation(200, 200);
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
			waitTask.setDuration(Double.parseDouble(argumentField.getText()));
			(new Thread(waitTask)).start();
		} else if ( source==turnToButton ) {
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

			SampleRobotApp.this.addKeyListener(this);
			SampleRobotApp.this.addMouseListener(this);

			while(!done) {
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("pause sleep interrupted");
				}
			}
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
			//sayDuration();
			startTime = System.currentTimeMillis();
			
			while(!done()) {
				// TODO update second text box here?
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("wait sleep interrupted");
				}
			}
		}
		
		private void sayDuration () {
			// cut off past two significant figures
			double roundedDuration = ((int) (duration*100)) / 100.0;
			if (speech != null) {
				speech.speak("Waiting " + roundedDuration + " seconds");
			}
		}

 		private boolean done() {
			return (System.currentTimeMillis() - startTime) / 1000 > duration;
		}

		public String toString() {
			return "wait task";
		}
	}

	class TurnToTask extends Task {
		
		Perceptor perceptor;

		TurnToTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			//showSC();
			robot.turnSonarsOn();

			perceptor = new Perceptor(robot);
			
			final int historySize = 50;
			final int bufferCapacity = NUM_SONARS*historySize;
			RingBuffer<Point2D> worldPoints
				= new RingBuffer<Point2D>(bufferCapacity);
			
			double[] sonars = new double[NUM_SONARS];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				
				pc.clear();
				
				RealPose2D robotRelWorld = new RealPose2D(robot.getPosX(), robot.getPosY(), robot.getHeading());
				pc.setReference(robotRelWorld);
				if (perceptor.getCurvature() != 0 && robot.getVelLeft() != 0) {
					for (int i = 0; i < sonars.length; i++){
						RealPoint2D obstacleRelSonar = new RealPoint2D(sonars[i],0.0);
						RealPose2D sonarRelRobot = sonarPose(i);
						Point2D obstacleRelRobot = sonarRelRobot.transform(obstacleRelSonar, null);
						Point2D obstacleRelWorld = robotRelWorld.transform(obstacleRelRobot, null);
						worldPoints.add(obstacleRelWorld);
					}
				}
				for (int i = 0; i < bufferCapacity; i++) {
					Point2D point = worldPoints.get(i);
					if (point == null) {
						continue;
					}
					Point2D pointRelRobot = robotRelWorld.inverseTransform(point, null);
					pc.addPoint(new RealPose2D(pointRelRobot, 0));
				}
				pc.drawAll(robot, ROBOT_RADIUS);
				
				robot.setVel(.25, .5);
				
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("visualization sleep interrupted");
				}
			}

			robot.turnSonarsOff();
			//hideSC();
		}

		public String toString() {
			return "sonar loop";
		}
	}
	
	private enum WallFollowState {
		GO, TURN, STOP
	}

	class GoToTask extends Task {
		
		Perceptor perceptor;
		Controller controller;

		GoToTask(TaskController tc) {
			super(tc);
		}
		
		private WallFollowState wallFollowState = WallFollowState.GO;

		public void taskRun() {
			//showSC();
			robot.turnSonarsOn();
			double[] sonars = new double[NUM_SONARS];
			
			perceptor = new Perceptor(robot);
			controller = new Controller(robot);
			
			final double ANGLE_TOLERANCE = 0.025;
			final double WALL_DISTANCE = ROBOT_RADIUS;
			
			double targetAngle = 0;
			double oldRightSonar = Double.MAX_VALUE;

			while(!shouldStop()) {				
				robot.updateState();
				robot.getSonars(sonars);
				
				// adjust angle if needed
				final double currentAngle = (robot.getHeading() + 2*PI) % (2*PI);
				targetAngle += 2*PI;
				targetAngle %= 2*PI;
				double diffAngle = targetAngle - currentAngle;
				if (abs(diffAngle) > PI) {
					diffAngle -= signum(diffAngle) * 2*PI;
				}
								
				switch (wallFollowState) {
				case GO:
					// check for followed wall to disappear
					double rightSonar = sonars[12];
					if (rightSonar > 3*oldRightSonar && oldRightSonar > 0) {
						System.out.println("should turn right");
						wallFollowState = WallFollowState.STOP;
						System.out.println("state = " + wallFollowState);
						targetAngle -= toRadians(90);
						break;
					}
					// check for wall ahead
					double frontSonar = sonars[0];
					double error = frontSonar - WALL_DISTANCE;
					if (abs(error) > 0.05) {
						final double SPEED_FACTOR = 1;
						double speed = error * SPEED_FACTOR;
						final double SLIGHT_SPIN_FACTOR = 0.25;
						double angleAdjust = diffAngle*SLIGHT_SPIN_FACTOR;
						controller.setVel(speed-angleAdjust, speed+angleAdjust);
					}
					else if (perceptor.getSpeed() == 0) {
						wallFollowState = WallFollowState.TURN;
						System.out.println("state = " + wallFollowState);
						
						targetAngle += toRadians(90);
					}
					else {
						controller.setVel(0, 0);
					}
					break;
				case TURN:
					if (abs(diffAngle) > ANGLE_TOLERANCE && abs(diffAngle - 2*PI) > ANGLE_TOLERANCE) {
						final double SPIN_FACTOR = 0.25;
						final double speed = diffAngle * SPIN_FACTOR;
						controller.setVel(-speed, speed);
					}
					else if (perceptor.getSpeed() == 0) {
						System.out.println("currentAngle = " + currentAngle);
						System.out.println("targetAngle = " + targetAngle);
						wallFollowState = WallFollowState.GO;
						System.out.println("state = " + wallFollowState);
					}
					else {
						controller.setVel(0, 0);
					}
					break;
				case STOP:
					if (perceptor.getSpeed() == 0) {
						wallFollowState = WallFollowState.TURN;
					}
					else {
						controller.setVel(0, 0);
					}
				}
				
				oldRightSonar = sonars[12];
				
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("\"stateful wanderer\" sleep interrupted");
				}
			}

			robot.turnSonarsOff();
			robot.setVel(0.0f, 0.0f);
			//hideSC();
		}

		public String toString() {
			return "\"both\"";
		}
	}
	
	private static final long serialVersionUID = 0;
}
