package edu.cmu.ri.mrpl;

import java.awt.Polygon;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.Comparator;
import java.util.Map;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import static edu.cmu.ri.mrpl.RobotModel.*;

public class Planner {
	/* returns array of n curvatures from -1 to 1 */
	public static double[] curvatureRange (int n)
	{
		double[] result = new double[n];
		
		double min = -5;
		double max = 5;
		double diff = max - min;
		
		for(int i = 0; i < n; i++)
		{
			result[i] = min + (double)i*diff/(n-1);
		}
		
		return result;
	}
	
	public static Area curvatureToArea (double curvature)
	{
		double radius = Math.abs(1.0/curvature);
		double arclength = 0.03;
		double theta = 360/(2.0*Math.PI*radius/arclength);
		Polygon wedge = new Polygon();
		double wedgeSize = 999;
		Area obstacle = null;
		
		double pathWidth = 0.001;
		
		wedge.addPoint(0, 0);
		RealPoint2D rightCorner = new RealPoint2D(wedgeSize, 0);
		wedge.addPoint((int) rightCorner.x, (int) rightCorner.y);
		if(curvature > 0) {
			RealPoint2D boundingPoint = new RealPoint2D(wedgeSize
					/ Math.tan(theta), wedgeSize);
			wedge.addPoint((int) boundingPoint.x, (int) boundingPoint.y);

			obstacle = new Area(new Ellipse2D.Double(-radius, 0,
					2 * radius, 2 * radius));
			obstacle.subtract(new Area(new Ellipse2D.Double(-radius, pathWidth,
					2 * (radius - pathWidth), 2 * (radius - pathWidth))));
			obstacle.intersect(new Area(wedge));
		}
		else if (curvature < 0) {
			RealPoint2D boundingPoint = new RealPoint2D(wedgeSize/Math.tan(theta), -wedgeSize);
			wedge.addPoint((int) boundingPoint.x, (int) boundingPoint.y);

			obstacle = new Area(new Ellipse2D.Double(-radius, -2*radius,
					2 * radius, 2 * radius));
			obstacle.subtract(new Area(new Ellipse2D.Double(-radius, pathWidth-2*radius,
					2 * (radius - pathWidth), 2 * (radius - pathWidth))));
			obstacle.intersect(new Area(wedge));
		}
		else {
			obstacle = new Area(new Rectangle2D.Double(0, -pathWidth, arclength*100, pathWidth*2));
		}
		return obstacle;
	}
	
	public static Pair<Double, Area>[] curvaturesToAreas (double[] curvatures)
	{
		Pair<Double, Area>[] result = new Pair[curvatures.length];
		
		for(int i = 0; i < curvatures.length; i++)
		{
			result[i] = new Pair<Double, Area>(curvatures[i], curvatureToArea(curvatures[i]));
		}
		return result;
	}
	
	public static Comparator<Pair<Double, Area>> curvPathComparator = new Comparator<Pair<Double, Area>>() {
		public int compare(Pair<Double, Area> o1, Pair<Double, Area> o2) {
			return 0;
		}
	};
	
	public static class CurvPathComparator implements Comparator<Pair<Double, Area>> {
		private double currentCurvature;
		private Area[] cSpaceObstacles;
		
		public CurvPathComparator (double currentCurvature, Area[] cSpaceObstacles) {
			this.currentCurvature = currentCurvature;
			this.cSpaceObstacles = cSpaceObstacles;
		}
		
		private double getCollisionDistance (Area path, Area obstacle) {
			Area intersection = (Area) path.clone();
			intersection.intersect(obstacle);
			Rectangle2D bound = intersection.getBounds2D();
			if (bound.isEmpty()) {
				return _maxSonarRange;
			}
			double x = bound.getCenterX();
			double y = bound.getCenterY();
			double dist = Math.sqrt(x*x + y*y);
			return dist;
		}
		
		public double getMinCollisionDistance (Area path) {
			double dist = Double.MAX_VALUE; // TODO change this to something else, maybe the distance just beyond the path?
			for (int i = 0; i < cSpaceObstacles.length; i++) {
				dist = Math.min(dist, getCollisionDistance(path, cSpaceObstacles[i]));
			}
			return dist;
		}
		
		public int compare(Pair<Double, Area> p1, Pair<Double, Area> p2) {
			Area a1 = p1.getSecond();
			Area a2 = p2.getSecond();
			
			double p1coll = getMinCollisionDistance(a1);
			double p2coll = getMinCollisionDistance(a2);
			
			double collisionDiff = Math.abs(p2coll - p1coll);
			final double TOLERANCE = 0.25;
			
			double p1diff = Math.abs(currentCurvature - p1.getFirst());
			double p2diff = Math.abs(currentCurvature - p2.getFirst());
			
			if (collisionDiff < TOLERANCE) {
				if (p1diff < p2diff) {
					return -1;
				}
				else {
					return 1;
				}
			}
			else if (p1coll > p2coll) {
				return -1;
			}
			else {
				return 1;
			}
		}
	}
	
	public static double[] getGoodCurves (double[] curves, Area[] curvesAreas, Area[] obstacles)
	{
		double[] result;
		int[] badIndices = new int[curves.length]; //default values are 0
		int count = 0;
		for(int i = 0; i < curvesAreas.length; i++)
		{
			for(Area ob: obstacles)
			{
				Area temp = ob;
				temp.intersect(curvesAreas[i]);
				if(!temp.isEmpty())
				{
					badIndices[i] = -1;
					count++;
				}
			}
		}
		result = new double[curves.length];
		
		for(int i = 0; i < curves.length; i++)
		{
			if(badIndices[i] == 0)
			{
				result[i] = curves[i];
			}
		}
		
		return result;
	}
	
	
	
	
	
/*
	public int obstacleInTheWay() {
		int sensorIndex = 0;
		Area[] obstacles = getCSpaceObstacles();
		//int turnDirection = openSide();
		
		//for (int i = 0; i != 9 * turnDirection; i += turnDirection) {
		for (int i = 0; i < 9; i++) {
			// just check the 9 sonars on the front since that's the direction we're moving
			sensorIndex = (16 - i + 4) % 16; 	

			if (pathIntersects(obstacles[sensorIndex])) {
				return sensorIndex;
			}
		}

		return -1;
	}

	public boolean pathIntersects(Area obstacle) {
		boolean doesIntersect = false;
		Area path;
		
		path = getPath();
		path.intersect(obstacle);
		
		if (path.isEmpty()) {
			doesIntersect = false;
		}
		else {
			doesIntersect = true;
		}

		return doesIntersect;
	}
	
	public Area getPath() {
		double k = getCurvature();
		double radius = 1 / k;
		double wiggleRoom = ROBOT_RADIUS;
		double farDistance = radius + wiggleRoom;
		double closeDistance = radius - wiggleRoom;
		Area path;
		
		if (k > 0) {
			path = new Area(new Ellipse2D.Double(-farDistance, -wiggleRoom, 2 * farDistance, 2 * farDistance));
			path.subtract(new Area(new Ellipse2D.Double(-closeDistance, wiggleRoom, 2 * closeDistance, 2 * closeDistance)));
			path.intersect(new Area(new Rectangle2D.Double(0, -wiggleRoom, farDistance, farDistance + wiggleRoom)));
		} else if (k < 0) {
			path = new Area(new Ellipse2D.Double(-farDistance, -2*farDistance + wiggleRoom, 2 * farDistance, -2 * farDistance));
			path.subtract(new Area(new Ellipse2D.Double(-closeDistance, -2*closeDistance - wiggleRoom, 2 * closeDistance, -2 * closeDistance)));
			path.intersect(new Area(new Rectangle2D.Double(0, -farDistance, farDistance, farDistance + wiggleRoom)));
		} else {
			path = new Area(new Rectangle2D.Double(0, -wiggleRoom, 5 * ROBOT_RADIUS, 2 * wiggleRoom));
		}
		
		return path;
	}
	
	// Returns 1 if there's more open space to the robot's right
	// returns -1 if there's more space to the left
	public int openSide() {
		double rightAverage = 0, leftAverage = 0;
		double[] sonars = new double[NUM_SONARS];

		robot.getSonars(sonars);

		for (int i = 1; i < 8; i++) {
			rightAverage += sonars[i] / 7;
		}
		for (int i = 15; i > 8; i--) {
			leftAverage += sonars[i] / 7;
		}

		return ((rightAverage > leftAverage) ? 1 : -1);
	}*/
}
