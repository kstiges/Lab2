package edu.cmu.ri.mrpl.maze;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;

public class MazeSolver {
	
	private MazeWorld world;
	private MazeState init;
	private Collection<MazeState> visited;
	private Map<MazeState, MazeState> parents;
	public List<MazeState> ourGolds;
	
	public MazeSolver (MazeWorld mw) {
		world = mw;
		init = mw.getInits().iterator().next();
		visited = new HashSet<MazeState>();
		parents = new HashMap<MazeState, MazeState>();
		findOurGolds();
	}
	
	private static MazeState getSymmetricGold(MazeState gold) {
		int dx, dy;
		switch (gold.dir) {
		case East:
			dx = 1;
			dy = 0;
			break;
			
		case West:
			dx = -1;
			dy = 0;
			break;
			
		case North:
			dx = 0;
			dy = 1;
			break;
			
		// case South:
		default:
			dx = 0;
			dy = -1;
			break;
		}
		
		return new MazeState(gold.x()+dx, gold.y()+dy, gold.dir().rear());
	}
	
	private void findOurGolds() {
		visited.clear();
		ourGolds = new LinkedList<MazeState>();
		
		MazeWorld scratch = new MazeWorld(world);
		scratch.removeAllGolds();
		
		Queue<MazeState> toVisit = new LinkedList<MazeState>();
		toVisit.add(init);
		
		while (!toVisit.isEmpty()) {
			MazeState current = toVisit.remove();
			visited.add(current);
			
			if (isGoalState(scratch, current, true)) {
				// current state and symmetric state are ours
				ourGolds.add(current);
				ourGolds.add(getSymmetricGold(current));
			}
			else if (isGoalState(world, current, true)) {
				// gold is in original maze, add it to our scratch maze
				scratch.addGold(current);
			}
			
			// add potential next states to the queue
			MazeState forward = world.act(current, MazeWorld.Action.GTNN);
			if (!visited.contains(forward)) {
				// if we didn't run into a wall going forward
				if (!forward.pos().equals(current.pos()))
				{
					toVisit.add(forward);
				}
			}

			MazeState left = world.act(current, MazeWorld.Action.TurnLeft);
			if (!visited.contains(left)) {
				toVisit.add(left);
			}

			MazeState right = world.act(current, MazeWorld.Action.TurnRight);
			if (!visited.contains(right)) {
				toVisit.add(right);
			}
		}
	}
	
	public List<MazeState> findPath (boolean findGold) {
		visited.clear();
		List<MazeState> result = new LinkedList<MazeState>();
		
		Queue<MazeState> toVisit = new LinkedList<MazeState>();
		toVisit.add(init);
		
		MazeState destState = null;
		MazeState ourGold = null;
		
		while (!toVisit.isEmpty()) {
			MazeState current = toVisit.remove();
			visited.add(current);
			// if this is a goal position
			if (isGoalState(world, current, findGold)) {
				boolean isOurs = ourGolds.contains(current);
				//System.out.println(isOurs);
				// if we're going for a drop, go ahead and find the path to it
				if (!isOurs) {
					destState = current;
					break;
				}
				else {
					if (ourGold == null) {
						ourGold = current;
					}
				}
			}
			
			//add potential next states to the queue
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
		
		if (findGold && destState == null) {
			destState = ourGold;
		}
		if (destState == null) {
			return null;
		}
		
		// extract path from ancestor map and return it
		MazeState current = destState;
		result.add(0, current);
		while (parents.containsKey(current)) {
			MazeState p = parents.get(current);
			result.add(0, p);
			current = p;
		}
		return result;
	}

	public static boolean isGoalState(MazeWorld mw, MazeState state, boolean findGold)
	{
		Set<MazeState> goals;
		if (findGold) {
			goals = mw.getFreeGolds();
		}
		else {
			goals = mw.getDrops();
		}
		for(MazeState o: goals)
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
		if (states == null) {
			return null;
		}
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
	
	
	public static void dedupPositions (List<MazeState> waypoints) {
		ListIterator<MazeState> iter = waypoints.listIterator();
		MazeState lastState = null;
		while (iter.hasNext()) {
			MazeState curState = iter.next();
			// if this state is on top of the previous one, remove the previous one
			if (lastState != null && curState.pos().equals(lastState.pos())) {
				iter.previous();
				iter.previous();
				iter.remove();
				iter.next();
			}
			lastState = curState;
		}
	}
	
	public static void main (String... args) {
		for (Direction dir : Direction.values()){
			System.out.println(dir);
			System.out.println(getSymmetricGold(new MazeState(2,2,dir)));
		}
	}
	
//	public static void main (String... args) {
//		List<MazeState> waypoints = new LinkedList<MazeState>();
//		waypoints.add(new MazeState(0,0,Direction.East));
//		waypoints.add(new MazeState(0,0,Direction.North));
//		waypoints.add(new MazeState(0,1,Direction.North));
//		waypoints.add(new MazeState(0,1,Direction.East));
//		
//		System.out.println(waypoints.size());
//		for (MazeState s : waypoints) {
//			System.out.println(s);
//		}
//		
//		System.out.println("DEDUPING");
//		dedupPositions(waypoints);
//		
//		System.out.println(waypoints.size());
//		for (MazeState s : waypoints) {
//			System.out.println(s);
//		}
//	}
}
