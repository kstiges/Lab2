package edu.cmu.ri.mrpl;

import static edu.cmu.ri.mrpl.RobotModel.*;

public class Controller {
	Robot robot;
	private final double MAX_SPEED = 0.5;
	
	public Controller (Robot robot) {
		this.robot = robot;
	}
	
	public void setCurvVel (double curv, double speed) {
		speed = Math.min(speed, MAX_SPEED);
		double angularVelocity = curv*speed;
		
		double rightVel = speed + angularVelocity*ROBOT_RADIUS;
		double leftVel = speed - angularVelocity*ROBOT_RADIUS;
		
		robot.setVel(leftVel, rightVel);
	}
	
	public void chooseCurvedPath (Perceptor perceptor, int obstacleDirection) {
		double k = perceptor.getCurvature();
		double k_scalingFactor = 5; // arbitrarily chosen
		int searchDirection = ((obstacleDirection < 8) ? 1 : -1);
		double MAX_VEL = .5; // arbitrarily chosen
		
		k += searchDirection*k_scalingFactor;
		
		// The speed should be a value inversely proportionally to the current curvature.
		// Thus, higher k means lower speed which should allow the robot to move more smoothly around
		// tighter turns instead of herky jerky stopping/turning.
		setCurvVel(k, Math.max(MAX_VEL, 1/k));
	}
}