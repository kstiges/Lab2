package edu.cmu.ri.mrpl.maze;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import javax.swing.JFileChooser;

import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;
import edu.cmu.ri.mrpl.util.Pair;

public class MazeSolver {
	
	private MazeWorld world;
	private MazeState init;
	private Collection<MazeState> visited;
	private Map<MazeState, MazeState> parents;
	
	public MazeSolver (MazeWorld mw) {
		world = mw;
		init = mw.getInits().iterator().next();
		visited = new HashSet<MazeState>();
		parents = new HashMap<MazeState, MazeState>();
	}
	
	public List<MazeState> findPath () {
		List<MazeState> result = new LinkedList<MazeState>();
		
		Queue<MazeState> toVisit = new LinkedList<MazeState>();
		toVisit.add(init);
		
		while (!toVisit.isEmpty()) {
			MazeState current = toVisit.remove();
			visited.add(current);
			// if this is a goal position
			if (isGoalState(world, current)) {
				// extract path from ancestor map and return it
				result.add(0, current);
				while (parents.containsKey(current)) {
					MazeState p = parents.get(current);
					result.add(0, p);
					current = p;
				}
				return result;
			}
			// otherwise, add potential next states to the queue
			else {
				MazeState forward = world.act(current, MazeWorld.Action.GTNN);
				if (!visited.contains(forward)) {
					// if we didn't run into a wall going forward
					if (!forward.pos().equals(current.pos()))
					{
						parents.put(forward, current);
						toVisit.add(forward);
					}
				}
				
				MazeState left = world.act(current, MazeWorld.Action.TurnLeft);
				if (!visited.contains(left)) {
					parents.put(left, current);
					toVisit.add(left);
				}
				
				MazeState right = world.act(current, MazeWorld.Action.TurnRight);
				if (!visited.contains(right)) {
					parents.put(right, current);
					toVisit.add(right);
				}
			}
		}
		
		return null;
	}

	public static boolean isGoalState(MazeWorld mw, MazeState state)
	{
		Object[] goals = mw.getGoals().toArray();
		for(Object o: goals)
		{
			MazeState ms = (MazeState) o;
			if(ms.equals(state))
			{
				return true;
			}
		}
		return false;
	}

	public static String statesToCommandsString (List<MazeState> states) {
		String result = "";
		for (int i = 0; i < states.size() - 1; i++) {
			MazeState from = states.get(i);
			MazeState to = states.get(i+1);
			
			Direction toDir = to.dir();
			Direction left = from.dir().left();
			Direction right = from.dir().right();
			
			if (toDir == left)
				result += "L";
			else if (toDir == right)
				result += "R";
			else
				result += "G";
		}
		return result;
	}
	
	public static void main (String... args) {
		List<MazeState> optimalPath;
		MazeWorld mazeWorld = null;
		JFileChooser chooser = new JFileChooser();
		int returnVal = chooser.showOpenDialog(chooser);
		if(returnVal == JFileChooser.APPROVE_OPTION) {
			System.out.println("You chose to open this file: " +
					chooser.getSelectedFile().getName());
		}
		try {
			mazeWorld = new MazeWorld(chooser.getSelectedFile().getAbsolutePath());
		} catch (IOException e) {
			System.err.println("Couldn't read maze file!");
		}

		optimalPath = new MazeSolver(mazeWorld).findPath();

		for (MazeState s : optimalPath) {
			System.out.println(s.pos());
		}
	}
}
