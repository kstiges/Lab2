package edu.cmu.ri.mrpl;

import static edu.cmu.ri.mrpl.RobotModel.*;
import static java.lang.Math.*;

public class Controller {
	Robot robot;
	public static final double MAX_SPEED = 0.6;
	
	public Controller (Robot robot) {
		this.robot = robot;
	}
	
	public void setVel (double leftVel, double rightVel) {
		// clamp values proportionally
		
		double maxVel = max(abs(leftVel), abs(rightVel));
		if (maxVel > MAX_SPEED) {
			double scaleFactor = MAX_SPEED / maxVel;
			leftVel *= scaleFactor;
			rightVel *= scaleFactor;
		}
		//System.err.println("setVel: " + leftVel + " " + rightVel);
		robot.setVel(leftVel, rightVel);
	}
	
	public void setCurvVel (double curv, double speed) {
		double angularVelocity = curv*speed;
		
		double rightVel = speed + angularVelocity*ROBOT_RADIUS;
		double leftVel = speed - angularVelocity*ROBOT_RADIUS;
		
		setVel(leftVel, rightVel);
	}

	public void stop() {
		setVel(0,0);		
	}
}