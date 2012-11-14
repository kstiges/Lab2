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
		return dist == 0 || (dist == 1 && a.dir().rear() == b.dir());
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