package edu.cmu.ri.mrpl.maze;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

import javax.swing.text.Position;

import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;
import static java.lang.Math.*;

public class MazeLocalizer {
	public static final double METERS_PER_INCH = 0.0254;
	public static final int WALL_INCHES = 29;
	public static final double WALL_METERS = WALL_INCHES * METERS_PER_INCH;
	public static final double CELL_RADIUS = WALL_METERS / 2;
	
	private RealPose2D initRelWorld;
	
	public MazeLocalizer (MazeWorld maze, boolean isWrong) {
		initRelWorld = isWrong
			? mazeStateToWorldPose(maze.getInits().iterator().next())
			: new RealPose2D();
	}
	
	// given the pose of the robot relative to its origin,
	// compute the cell coordinates of the robot pose
	public RealPose2D fromInitToCell (RealPose2D robotRelInit) {
		return fromWorldToCell(fromInitToWorld(robotRelInit));
	}
	
	// given the pose of the robot relative to its origin,
	// compute the pose of the robot in the maze world
	public RealPose2D fromInitToWorld (RealPose2D robotRelInit) {
		return RealPose2D.multiply(initRelWorld, robotRelInit);
	}
	
	public Point2D transformInitToWorld (Point2D pointRelInit) {
		return initRelWorld.transform(pointRelInit, null);
	}
	
	// converts a pose in the maze world to cell coordinates
	public static RealPose2D fromWorldToCell (RealPose2D worldPose) {
		double x = (worldPose.getX() - CELL_RADIUS) / WALL_METERS;
		double y = (worldPose.getY() - CELL_RADIUS) / WALL_METERS;
		double theta = Angle.normalize(worldPose.getTh());
		/*if (theta < 0) {
			theta += 2*PI;
		}*/
		theta /= PI/2;
		return new RealPose2D(x, y, theta);
	}
	
	public static RealPose2D fromMazeToWorld (RealPose2D cellPose) {
		double x = cellPose.getX()*WALL_METERS + CELL_RADIUS;
		double y = cellPose.getY()*WALL_METERS + CELL_RADIUS;
		double theta = Angle.normalize(cellPose.getTh());
		/*if (theta < 0) {
			theta += 2*PI;
		}*/
		theta /= PI/2;
		return new RealPose2D(x, y, theta);
	}
	
	// converts a (x, y, dir) maze state
	// to an (x, y, theta) pose in the maze world
	public static RealPose2D mazeStateToWorldPose (MazeState state) {
		double x = CELL_RADIUS + (state.x() * WALL_METERS);
		double y = CELL_RADIUS + (state.y() * WALL_METERS);
		double theta = Angle.normalize(state.dir.ordinal() * (PI / 2));
		return new RealPose2D(x, y, theta);
	}
	
	public static List<RealPose2D> statesToPoses (List<MazeState> states) {
		List<RealPose2D> result = new LinkedList<RealPose2D>();
		
		RealPoint2D lastPosition = null;
		
		for (MazeState s : states) {
			RealPose2D pose = mazeStateToWorldPose(s);
			
			// skip pairs of poses in the same position
			RealPoint2D position = pose.getPosition();
			if (position.equals(lastPosition))
				result.remove(result.size() - 1);
			lastPosition = position;
			
			// TODO generalize this
			result.add(new RealPose2D(pose.getX()-CELL_RADIUS, pose.getY()-CELL_RADIUS, pose.getTh()));
		}
		
		return result;
	}
	
	/*
	public static void main (String... args) {
		int[] coords = new int[]{
				0,0,
				5,0,
				5,1,
				2,1,
				2,2,
				1,2,
				1,1,
				0,1,
				0,3,
				5,3,
				5,0,
				0,0
		};
		
		for (int i = 0; i < coords.length; i += 2) {
			RealPose2D pose = mazeStateToWorldPose(new MazeState(coords[i], coords[i+1], Direction.East));
			System.out.printf("%f,%f,%f;", pose.getX()-CELL_RADIUS, pose.getY()-CELL_RADIUS, pose.getTh());
		}
	}
	*/
}