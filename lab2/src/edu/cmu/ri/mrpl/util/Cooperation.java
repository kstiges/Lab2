package edu.cmu.ri.mrpl.util;

import java.awt.Point;
import java.util.List;

import edu.cmu.ri.mrpl.maze.MazeSolver;
import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;
import static java.lang.Math.*;

public class Cooperation {
	
	// waypoints collide if they are in the same position
	// or if they are adjacent but pointing towards each other
	public static boolean waypointsCollide (MazeState a, MazeState b) {
		Point aPos = new Point(a.x(), a.y());
		Point bPos = new Point(b.x(), b.y());
		double dist = aPos.distance(bPos);
		
		// states are in the same position
		if (dist == 0) {
			return true;
		}
		// states are not adjacent
		else if (dist > 1) {
			return false;
		}
		
		// states are adjacent on the same row
		if (aPos.y == bPos.y) {
			MazeState leftState = aPos.x < bPos.x ? a : b;
			MazeState rightState = leftState == a ? b : a;
			// states point towards each other
			if (leftState.dir() == Direction.East && rightState.dir() == Direction.West) {
				return true;
			}
		}
		
		// states are adjacent on the same column
		if (aPos.x == bPos.x) {
			MazeState topState = aPos.y > bPos.y ? a : b;
			MazeState bottomState = topState == a ? b : a;
			// states point towards each other
			if (topState.dir() == Direction.South && bottomState.dir() == Direction.North) {
				return true;
			}
		}
		
		return false;
	}
	
	public static boolean waypointListsCollide(List<MazeState> waypointsA,
			List<MazeState> waypointsB) {
		
		// dedup in case it hasn't already been done
		MazeSolver.dedupPositions(waypointsA);
		MazeSolver.dedupPositions(waypointsB);
		
		int minSize = min(waypointsA.size(), waypointsB.size());
		for (int i = 0; i < minSize; i++) {
			MazeState a = waypointsA.get(i);
			MazeState b = waypointsB.get(i);
			if (waypointsCollide(a, b)) {
				return true;
			}
		}
		return false;
	}
}