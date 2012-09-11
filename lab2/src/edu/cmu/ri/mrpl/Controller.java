package edu.cmu.ri.mrpl;

import static edu.cmu.ri.mrpl.RobotModel.*;
import static java.lang.Math.*;

public class Controller {
	Robot robot;
	public static final double MAX_SPEED = 0.5;
	
	public Controller (Robot robot) {
		this.robot = robot;
	}
	
	public void setVel (double leftVel, double rightVel) {
		// clamp values
		leftVel = min(leftVel, MAX_SPEED);
		rightVel = min(rightVel, MAX_SPEED);
		leftVel = max(leftVel, -MAX_SPEED);
		rightVel = max(rightVel, -MAX_SPEED);
		robot.setVel(leftVel, rightVel);
	}
	
	public void setCurvVel (double curv, double speed) {
		double angularVelocity = curv*speed;
		
		double rightVel = speed + angularVelocity*ROBOT_RADIUS;
		double leftVel = speed - angularVelocity*ROBOT_RADIUS;
		
		setVel(leftVel, rightVel);
	}
}