package edu.cmu.ri.mrpl;

import java.awt.Polygon;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import static edu.cmu.ri.mrpl.RobotModel.*;

public class Perceptor {
	Robot robot;
	
	public static final double SONAR_RADIANS = 2 * Math.PI / NUM_SONARS;
	public static final double SONAR_WIDTH = Math.toRadians(25);
	public static final double PADDING = 1*ROBOT_RADIUS;
	
	private RealPose2D correctedPose = new RealPose2D();
	private RealPose2D odometryPoseAtCorrection = new RealPose2D();

	public Perceptor(Robot robot) {
		this.robot = robot;
	}
	
	public void setCorrectedPose (RealPose2D corrected) {
		correctedPose = corrected;
		odometryPoseAtCorrection = getWorldPose();
	}
	
	public RealPose2D getCorrectedPose () {
		RealPose2D odomOldToNew = RealPose2D.multiply(odometryPoseAtCorrection.inverse(), getWorldPose());
		return RealPose2D.multiply(correctedPose, odomOldToNew);
	}
	
	// returns the pose of the robot in the world frame
	public RealPose2D getWorldPose () {
		return new RealPose2D(robot.getPosX(), robot.getPosY(), robot.getHeading());
	}
	
	// returns the pose of the robot in the argument frame (which is given relative to the world frame)
	public RealPose2D getRelPose (RealPose2D frame) {
		RealPose2D robotRelWorld = getWorldPose();
		RealPose2D inverseFrame = frame.inverse();
		return RealPose2D.multiply(inverseFrame, robotRelWorld);
	}

	public RealPoint2D[] getSonarObstacles() {
		RealPoint2D[] result = new RealPoint2D[NUM_SONARS];

		double[] sonars = new double[NUM_SONARS];
		robot.getSonars(sonars);

		for (int i = 0; i < NUM_SONARS; i++) {
			double sonarDistance = sonars[i];
			double robotDistance = sonarDistance + ROBOT_RADIUS;
			double radians = i * SONAR_RADIANS;

			double objX = robotDistance * Math.cos(radians);
			double objY = robotDistance * Math.sin(radians);

			result[i] = new RealPoint2D(objX, objY);
		}

		return result;
	}

	public Area[] getCSpaceObstacles() {
		Area[] result = new Area[NUM_SONARS];
		double[] sonars = new double[NUM_SONARS];
		robot.getSonars(sonars);
		
		for (int i = 0; i < NUM_SONARS; i++) {
			double sonarDistance = sonars[i];
			double robotDistance = sonarDistance + ROBOT_RADIUS;
			double closeDistance = robotDistance - PADDING;
			double farDistance = robotDistance + PADDING;

			double sonarAngle = i * SONAR_RADIANS;
			double radiusAngle = Math.atan2(PADDING, sonarDistance);
			double leftEdgeAngle = sonarAngle + SONAR_WIDTH / 2 + radiusAngle;
			double rightEdgeAngle = sonarAngle - SONAR_WIDTH / 2 - radiusAngle;

			Polygon wedge = new Polygon();
			wedge.addPoint(0, 0);
			double wedgeSize = 999;

			RealPoint2D leftCorner = new RealPoint2D(wedgeSize
					* Math.cos(leftEdgeAngle), wedgeSize
					* Math.sin(leftEdgeAngle));
			wedge.addPoint((int) leftCorner.x, (int) leftCorner.y);

			RealPoint2D rightCorner = new RealPoint2D(wedgeSize
					* Math.cos(rightEdgeAngle), wedgeSize
					* Math.sin(rightEdgeAngle));
			wedge.addPoint((int) rightCorner.x, (int) rightCorner.y);

			Area obstacle = new Area(new Ellipse2D.Double(-farDistance,
					-farDistance, 2 * farDistance, 2 * farDistance));
			obstacle.subtract(new Area(new Ellipse2D.Double(-closeDistance,
					-closeDistance, 2 * closeDistance, 2 * closeDistance)));
			obstacle.intersect(new Area(wedge));

			result[i] = obstacle;
		}

		return result;
	}

	public double getCurvature() {
		double vel = getSpeed();
		if (vel == 0) {
			return 0;
		}
		double angularVel = (robot.getVelRight() - robot.getVelLeft())
				/ (2 * ROBOT_RADIUS);

		return angularVel / vel;
	}
	
	public double getSpeed() {
		double leftVel = robot.getVelLeft();
		double rightVel = robot.getVelRight();
		double vel = (leftVel + rightVel) / 2;
		return vel;
	}
}
