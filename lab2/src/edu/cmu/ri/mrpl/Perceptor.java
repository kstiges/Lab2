package edu.cmu.ri.mrpl;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Rectangle2D.Double;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import static edu.cmu.ri.mrpl.RobotModel.*;

public class Perceptor {
	Robot robot;
	private final double SONAR_RADIANS = 2 * Math.PI / NUM_SONARS;
	private final double SONAR_WIDTH = Math.toRadians(25);

	public Perceptor(Robot robot) {
		this.robot = robot;
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
			double closeDistance = sonarDistance; // do we need to take the
													// radius of the robot into
													// consideration here/
			double farDistance = closeDistance + 2 * ROBOT_RADIUS;

			double sonarAngle = i * SONAR_RADIANS;
			double leftEdgeAngle = sonarAngle + SONAR_WIDTH / 2;
			double rightEdgeAngle = leftEdgeAngle - SONAR_WIDTH;

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
		double leftVel = robot.getVelLeft();
		double rightVel = robot.getVelRight();
		if (leftVel == 0 && rightVel == 0) {
			return 0;
		}
		double vel = (leftVel + rightVel) / 2;
		double angularVel = (rightVel - leftVel) / (2 * ROBOT_RADIUS);

		return angularVel / vel;
	}
}
