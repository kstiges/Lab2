package edu.cmu.ri.mrpl;

import java.awt.Polygon;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.util.Pair;
import static edu.cmu.ri.mrpl.RobotModel.*;
import static java.lang.Math.*;

public class Planner {
	/* returns array of n curvatures from -1 to 1 */
	public static double[] curvatureRange (int n)
	{
		double[] result = new double[n];
		
		double range = 3;
		double min = -range;
		double max = range;
		double diff = max - min;
		
		for(int i = 0; i < n; i++)
		{
			result[i] = min + (double)i*diff/(n-1);
			System.out.println(i + "th curvature: " + result[i]);
		}
		
		return result;
	}
	
	public static Area curvatureToArea (double curvature)
	{
		double radius = abs(1.0/curvature);
		double arclength = 0.03;
		double theta = 360/(2.0*PI*radius/arclength);
		Polygon wedge = new Polygon();
		double wedgeSize = 999;
		Area obstacle = null;
		
		double pathWidth = 0.001;
		
		wedge.addPoint(0, 0);
		RealPoint2D rightCorner = new RealPoint2D(wedgeSize, 0);
		wedge.addPoint((int) rightCorner.x, (int) rightCorner.y);
		if(curvature > 0) {
			RealPoint2D boundingPoint = new RealPoint2D(wedgeSize
					/ tan(theta), wedgeSize);
			wedge.addPoint((int) boundingPoint.x, (int) boundingPoint.y);

			obstacle = new Area(new Ellipse2D.Double(-radius, 0,
					2 * radius, 2 * radius));
			obstacle.subtract(new Area(new Ellipse2D.Double(-radius, pathWidth,
					2 * (radius - pathWidth), 2 * (radius - pathWidth))));
			obstacle.intersect(new Area(wedge));
		}
		else if (curvature < 0) {
			RealPoint2D boundingPoint = new RealPoint2D(wedgeSize/tan(theta), -wedgeSize);
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
		obstacle.intersect(new Area(new Rectangle2D.Double(0, -_maxSonarRange, _maxSonarRange, 2*_maxSonarRange)));
		return obstacle;
	}
	
	public static Pair<Double, Area>[] curvaturesToAreas (double[] curvatures)
	{
		@SuppressWarnings("unchecked")
		Pair<Double, Area>[] result = new Pair[curvatures.length];
		
		for(int i = 0; i < curvatures.length; i++)
		{
			result[i] = new Pair<Double, Area>(curvatures[i], curvatureToArea(curvatures[i]));
			System.out.println(result[i].getSecond().getBounds2D());
		}
		return result;
	}
	
	public static class CurvPathComparator implements Comparator<Pair<Double, Area>> {
		//private double currentCurvature;
		private Area[] cSpaceObstacles;
		private Map<Area, Double> minCollisionDistances;
		
		public CurvPathComparator (double currentCurvature, Area[] cSpaceObstacles) {
			//this.currentCurvature = currentCurvature;
			this.cSpaceObstacles = cSpaceObstacles;
			minCollisionDistances = new HashMap<Area, Double>();
		}
		
		private double getCollisionDistance (double k, Area path, Area obstacle) {
			// check to see if path circle is smaller than distance to obstacle
			if (k != 0) {
				double radius = abs(1/k);
				Rectangle2D bound = obstacle.getBounds2D();
				double x = min(abs(bound.getMaxX()), abs(bound.getMinX()));
				double y = min(abs(bound.getMaxY()), abs(bound.getMinY()));
				double dist = sqrt(x*x + y*y);
				if (2*radius < dist)
					return _maxSonarRange;
			}
			Area intersection = (Area) path.clone();
			intersection.intersect(obstacle);
			Rectangle2D bound = intersection.getBounds2D();
			if (bound.isEmpty()) {
				return _maxSonarRange;
			}
			// find distance to corner closest to origin
			double x = min(abs(bound.getMaxX()), abs(bound.getMinX()));
			double y = min(abs(bound.getMaxY()), abs(bound.getMinY()));
			double dist = sqrt(x*x + y*y);
			return dist;
		}
		
		public double getMinCollisionDistance (double k, Area path) {
			if (minCollisionDistances.containsKey(path)) {
				return minCollisionDistances.get(path);
			}
			double dist = _maxSonarRange;
			for (int i = 0; i < cSpaceObstacles.length; i++) {
				dist = min(dist, getCollisionDistance(k, path, cSpaceObstacles[i]));
			}
			minCollisionDistances.put(path, dist);
			return dist;
		}
		
		public int compare(Pair<Double, Area> p1, Pair<Double, Area> p2) {
			Area a1 = p1.getSecond();
			Area a2 = p2.getSecond();
			
			double p1coll = getMinCollisionDistance(p1.getFirst(), a1);
			double p2coll = getMinCollisionDistance(p2.getFirst(), a2);
			
			double collisionDiff = abs(p2coll - p1coll);
			final double TOLERANCE = 0.25;
			
			double p1diff = abs(p1.getFirst());
			double p2diff = abs(p2.getFirst());
			
			if (collisionDiff < TOLERANCE) {
				if (p1diff < p2diff) {
					return -1;
				}
				else if (p2diff < p1diff) {
					return 1;
				}
				else {
					return 0;
				}
			}
			else if (p1coll > p2coll) {
				return -1;
			}
			else if (p2coll > p1coll) {
				return 1;
			}
			else {
				return 0;
			}
		}
	}
	
	/*
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
