package edu.cmu.ri.mrpl.comm;

import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;

public class Serializer {
	public static final String DELIM = ",";
	
	public static String serialize (MazeState state) {
		return state.x() + DELIM + state.y() + DELIM + state.dir().ordinal();
	}
	
	public static MazeState parseMazeState (String s) {
		String[] fields = s.split(DELIM);
		int x = Integer.parseInt(fields[0]);
		int y = Integer.parseInt(fields[1]);
		int dirOrd = Integer.parseInt(fields[2]);
		Direction dir = Direction.values()[dirOrd];
		return new MazeState(x, y, dir);
	}
}
