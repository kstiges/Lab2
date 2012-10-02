package edu.cmu.ri.mrpl.maze;

import java.awt.geom.Point2D;
import java.util.Scanner;

import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;
import static java.lang.Math.*;

public class MazeLocalizer {
	public static final double METERS_PER_INCH = 0.0254;
	public static final int WALL_INCHES = 29;
	public static final double WALL_METERS = WALL_INCHES * METERS_PER_INCH;
	public static final double CELL_RADIUS = WALL_METERS / 2;
	
	private RealPose2D initRelWorld;
	
	public MazeLocalizer (MazeWorld maze) {
		initRelWorld = 
			mazeStateToWorldPose(maze.getInits().iterator().next());
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
		if (theta < 0) {
			theta += 2*PI;
		}
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
	
	public static void main (String... args) {
		MazeWorld world = new MazeWorld(5, 3);
		world.addInit(2, 1, Direction.North);
		
		MazeLocalizer local = new MazeLocalizer(world);
		Scanner scanner = new Scanner(System.in);
		while (true) {
			System.out.println("Enter x:");
			double x = scanner.nextDouble();
			
			System.out.println("Enter y:");
			double y = scanner.nextDouble();
			
			System.out.println("Enter theta:");
			double theta = scanner.nextDouble();
			
			System.out.println(
				local.fromInitToCell(new RealPose2D(x, y, theta)));
		}
	}
}