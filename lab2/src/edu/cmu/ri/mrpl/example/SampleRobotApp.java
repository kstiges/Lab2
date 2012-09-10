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
import java.util.Arrays;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import javax.swing.*;

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Planner.CurvPathComparator;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import static edu.cmu.ri.mrpl.RobotModel.*;

public class SampleRobotApp extends JFrame implements ActionListener, TaskController {

	private Robot robot;
	private SonarConsole sc;
	private JFrame scFrame;

	private JButton connectButton;
	private JButton disconnectButton;

	private JButton reactiveWandererButton;
	private JButton statefulVisualizationButton;
	private JButton statefulWanderer;

	private JButton stopButton;
	private JButton quitButton;

	private ReactiveWandererTask reactiveWandererTask;
	private StatefulVisualizationTask statefulVisualizationTask;
	private StatefulWandererTask statefulWandererTask;
	
	private PointsConsole pc;
	static final int DEFAULT_ROOM_SIZE = 4;
	
	private Task curTask = null;

	public SampleRobotApp() {
		super("Kick Ass Sample Robot App");

		connectButton = new JButton("connect");
		disconnectButton = new JButton("disconnect");

		reactiveWandererButton = new JButton("Run Reactive Wanderer!");
		statefulVisualizationButton = new JButton("Run Stateful Visualization!");
		statefulWanderer = new JButton("Run Stateful Wanderer!");
		stopButton = new JButton(">> stop <<");
		quitButton = new JButton(">> quit <<");

		connectButton.addActionListener(this);
		disconnectButton.addActionListener(this);

		reactiveWandererButton.addActionListener(this);
		statefulVisualizationButton.addActionListener(this);
		statefulWanderer.addActionListener(this);
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
		box.add(reactiveWandererButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(statefulVisualizationButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(statefulWanderer);

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
		this.setVisible(true);

		// construct the SonarConsole, but don't make it visible
		scFrame = new JFrame("Sonar Console");
		scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		sc = new SonarConsole();
		scFrame.add(sc);
		scFrame.pack();
		
		pc = new PointsConsole(768-640, 1024-640, 640, 640);
		pc.setWorldViewport(-1.0 * DEFAULT_ROOM_SIZE, -1.0 * DEFAULT_ROOM_SIZE, 1.0 * DEFAULT_ROOM_SIZE - 0.5, 1.0 * DEFAULT_ROOM_SIZE - 0.5);

		// construct tasks
		reactiveWandererTask = new ReactiveWandererTask(this);
		statefulVisualizationTask = new StatefulVisualizationTask(this);
		statefulWandererTask = new StatefulWandererTask(this);
		
		Frame f = new Frame();
		f.setTitle("Points Console");
		
		/* TODO figure this out
		RotatePanel rp = new RotatePanel();
		rp.add(pc.getPanel());
		rp.setSize(pc.getPanel().getSize());
		f.add(rp);
		/*/
		f.add(pc.getPanel());
		//*/
		
		f.setSize(pc.getPanel().getSize());
		f.setVisible(true);
		f.setLocation(200, 200);

		//*
		robot = new SimRobot();
		/*/
		robot = new ScoutRobot();
		//*/
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
		} else if ( source==reactiveWandererButton ) {
			(new Thread(reactiveWandererTask)).start();
		} else if ( source==statefulVisualizationButton ) {
			(new Thread(statefulVisualizationTask)).start();
		} else if ( source==statefulWanderer ) {
			(new Thread(statefulWandererTask)).start();
		}
	}

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
	
	class ReactiveWandererTask extends Task {
		
		Perceptor perceptor;
		Controller controller;

		ReactiveWandererTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			robot.turnSonarsOn();
			
			perceptor = new Perceptor(robot);
			controller = new Controller(robot);
			
			final double[] curves = Planner.curvatureRange(11);
			final Pair<Double, Area>[] curvesAreas = Planner.curvaturesToAreas(curves);

			while(!shouldStop()) {
				
				robot.updateState();
				Area[] cSpaceObstacles = perceptor.getCSpaceObstacles();
				
				/* XXX draws obstacles and paths
				pc.clear();
				double scaleFactor = 50;
				int width = pc.getPanel().getWidth();
				int height = pc.getPanel().getHeight();
				for (int i = 0; i < NUM_SONARS; i++) {
					Area a = (Area) cSpaceObstacles[i].clone();

					a.transform(AffineTransform.getScaleInstance(scaleFactor, -scaleFactor));
					a.transform(AffineTransform.getTranslateInstance(width/2, height/2));
					((Graphics2D) pc.getPanel().getGraphics()).draw(a);
				}
				
				for (int i = 0; i < curves.length; i++)
				{
					Area c = (Area) curvesAreas[i].getSecond().clone();
					
					c.transform(AffineTransform.getScaleInstance(scaleFactor, -scaleFactor));
					c.transform(AffineTransform.getTranslateInstance(width/2, height/2));
					((Graphics2D) pc.getPanel().getGraphics()).draw(c);
				}
				//*/
				
				double currentCurvature = perceptor.getCurvature();
				CurvPathComparator cmp = new Planner.CurvPathComparator(currentCurvature, cSpaceObstacles);
				Arrays.sort(curvesAreas, cmp);
				
				double maxMinCollisionDistance = -1;
				for (Pair<Double, Area> curvPath : curvesAreas) {
					maxMinCollisionDistance = Math.max(maxMinCollisionDistance,
							cmp.getMinCollisionDistance(curvPath.getSecond()));
				}
				
				if (maxMinCollisionDistance > ROBOT_RADIUS) {
					int curvIndex = 0;
					final double vel = 0.25;
					controller.setCurvVel(curvesAreas[curvIndex].getFirst(), vel);
					//System.out.println("paths dont suck, collision distance is: " + minMinCollisionDistance);
				}
				else {
					final double spinSpeed = 0.1;
					// turn around because all the paths suck
					controller.setVel(spinSpeed, -spinSpeed);
					System.out.println("turning around because all the paths suck."
							+ " Collision Distance is: " + maxMinCollisionDistance);
				}
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("reactive wanderer sleep interrupted");
				}
			}
			
			robot.turnSonarsOff();
			robot.setVel(0,0);
		}

		public String toString() {
			return "sample program";
		}
	}

	class StatefulVisualizationTask extends Task {

		StatefulVisualizationTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			//showSC();
			robot.turnSonarsOn();
			
			double[] sonars = new double[NUM_SONARS];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				
				pc.setReference(new RealPose2D(robot.getPosX(),robot.getPosY(),robot.getHeading()));
				for( int i = 0; i < sonars.length; ++i){
					RealPose2D sonar = new RealPose2D(sonars[i],0.0,0.0);
					RealPose2D sonarToRobot = sonarPose(i);
					pc.addPoint(RealPose2D.multiply(sonarToRobot, sonar));
				}
				pc.drawAll(robot, RobotModel.ROBOT_RADIUS);
				
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

	class StatefulWandererTask extends Task {

		StatefulWandererTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			showSC();
			robot.turnSonarsOn();

			while(!shouldStop()) {
				robot.updateState();
				
				try {
					Thread.sleep(50);
				} catch(InterruptedException iex) {
					System.out.println("\"stateful wanderer\" sleep interrupted");
				}
			}

			robot.turnSonarsOff();
			robot.setVel(0.0f, 0.0f);
			hideSC();
		}

		public String toString() {
			return "\"both\"";
		}
	}
	
	private static final long serialVersionUID = 0;
}
