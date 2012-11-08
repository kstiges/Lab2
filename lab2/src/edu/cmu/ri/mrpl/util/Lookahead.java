package edu.cmu.ri.mrpl.util;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

import edu.cmu.ri.mrpl.kinematics2D.LineSegment;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.kinematics2D.Vector2D;

public class Lookahead {
	
	static int currentSegment = 0;
	
	public static void zeroCurrentSegment () {
		currentSegment = 0;
	}
	
	public static void main (String... args) {
		// make a unit square
		List<Line2D> pathSegments = new LinkedList<Line2D>();
		pathSegments.add(new Line2D.Double(0, 0, 1, 0));
		pathSegments.add(new Line2D.Double(1, 0, 1, 1));
		pathSegments.add(new Line2D.Double(1, 1, 0, 1));
		pathSegments.add(new Line2D.Double(0, 1, 0, .1)); // almost closed
		
		// put the robot at the origin
		Point2D robotRelWorld = new Point2D.Double(0, 0);
		
		//System.out.println(findClosestPoint(pathSegments, robotRelWorld, null));
		
		// prompt user for lookahead distances and print the lookahead point and segment
		//*
		System.out.println("hey");
		Scanner input = new Scanner(System.in);
		while (input.hasNextDouble()) {
			double dist = input.nextDouble();
			Point2D lookaheadPoint = new Point2D.Double();
			int lookaheadIndex = findLookaheadPoint(pathSegments, robotRelWorld, dist, lookaheadPoint);
			System.out.println(lookaheadPoint + " on segment " + lookaheadIndex);
		}
		//*/
	}
	
	public static List<Line2D> posesToPath (List<RealPose2D> poses) {
		List<Line2D> pathSegments = new ArrayList<Line2D>();
		for (int i = 1; i < poses.size(); i++) {
			pathSegments.add(new Line2D.Double(poses.get(i-1).getPosition(),
					poses.get(i).getPosition()));
		}
		return pathSegments;
	}
	
	public static int findClosestPoint (List<Line2D> pathSegments, Point2D robotRelWorld, RealPoint2D retClosestPoint) {
		RealPoint2D closestPoint = new RealPoint2D();
		double minDistSquared = 0.5;
		int closestSegmentIndex = -1;
		double closestDistSquared = Double.MAX_VALUE;
		for (int i = currentSegment; i < pathSegments.size(); i++) {
			Line2D segment = pathSegments.get(i);
			RealPoint2D tmp = new RealPoint2D();
			double tmpDistSquared = LineSegment.closestPointOnLineSegment(segment, robotRelWorld, tmp);
			if (tmpDistSquared <= closestDistSquared) {
				closestPoint.setLocation(tmp);
				closestSegmentIndex = i;
				closestDistSquared = tmpDistSquared;
				currentSegment = i;
			}
			if (closestDistSquared <= minDistSquared) {
				if (retClosestPoint != null) {
					retClosestPoint.setLocation(closestPoint);
				}
				return closestSegmentIndex;
			}
		}
		/*
		if (oldClosestSegment != null && !closestSegment.equals(oldClosestSegment)) {
			System.out.printf("CHANGING SEGMENTS FROM %d to %d\n",
					pathSegments.indexOf(oldClosestSegment), pathSegments.indexOf(closestSegment));
		}
		*/
		if (retClosestPoint != null) {
			retClosestPoint.setLocation(closestPoint);
		}
		
		return closestSegmentIndex;
	}
	
	public static int findLookaheadPoint (List<Line2D> pathSegments, Point2D robotRelWorld, double lookaheadDistance, Point2D retLookaheadPoint) {
		// find closest point and segment
		RealPoint2D closestPoint = new RealPoint2D();
		int closestSegmentIndex = findClosestPoint(pathSegments, robotRelWorld, closestPoint);
		//System.err.println(closestSegmentIndex);

		if (closestSegmentIndex < 0){
			return -1;
		}
		
		// try every segment until the lookahead distance is reached
		for (int i = closestSegmentIndex; i < pathSegments.size(); i++) {
			// chop off the part of the segment before the closest point
			Line2D segment = new Line2D.Double(closestPoint, pathSegments.get(i).getP2());
			Point2D segmentEnd = segment.getP2();
			double segmentLength = closestPoint.distance(segmentEnd);
			
			// determine whether lookahead goes beyond this segment
			// if so, adjust closest point and lookahead distance accordingly
			if (segmentLength < lookaheadDistance) {
				lookaheadDistance -= segmentLength;
				closestPoint = new RealPoint2D(segmentEnd.getX(), segmentEnd.getY());
				//System.out.println("LOOKAHEAD ON SEGMENT AFTER " + i);
				continue;
			}
			// otherwise, the lookahead point is on this segment, so find it
			else {
				ArrayList<Vector2D> points = new ArrayList<Vector2D>();
				if (LineSegment.radialPointsOnLineSegment(segment, lookaheadDistance, closestPoint, points)) {
					if (points.size() != 1) {
						System.err.println("FOUND MULTIPLE CLOSEST POINTS");
					}
					Vector2D lookaheadPoint = points.get(0);
					if (retLookaheadPoint != null) {
						retLookaheadPoint.setLocation(lookaheadPoint);
					}
					currentSegment = i;
					return i;
				}
			}
		}
		// handles case where lookahead goes off the end of the path
		if (retLookaheadPoint != null) {
			retLookaheadPoint.setLocation(pathSegments.get(pathSegments.size()-1).getP2());
		}
		return pathSegments.size()-1;
	}
}
