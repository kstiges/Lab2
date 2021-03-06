package edu.cmu.ri.mrpl.maze;

import java.awt.geom.Point2D;

import edu.cmu.ri.mrpl.maze.MazeWorld.Direction;
import static edu.cmu.ri.mrpl.maze.MazeWorld.Direction.*;

public class ProbabilisticWallGrid {
	// Hold hits/misses for horizontal and vertical walls separately.
	// Indexed by x, then y, then hits (0) or misses (1)
	private int[][][] horizontalWalls;
	private int[][][] verticalWalls;
	private int mazeWidth, mazeHeight; // in cells

	// To be said to exist, a wall must have at least this probability of existence.
	// Adjust this as needed.
	private static final double EXIST_PROB_THRESHOLD = 0.75;

	// Constructs a wall grid for a maze of given width/height
	public ProbabilisticWallGrid (int width, int height) {		
		horizontalWalls = new int[width][height+1][2];
		verticalWalls = new int[width+1][height][2];
		mazeWidth = width - 1;
		mazeHeight = height - 1;
	}
	
	public ProbabilisticWallGrid(MazeWorld mw) {
		this(mw.getWidth(), mw.getHeight());
	}
	
	public int getWidth() {
		return horizontalWalls.length;
	}
	
	public int getHeight() {
		return verticalWalls[0].length;
	}

	// Indicate a sonar hit on the wall at the given x, y, and direction
	public void hitWall (int cellX, int cellY, Direction wallDirection) {
		getWall(cellX, cellY, wallDirection)[0]++;
	}
	
	public void hitNearestWall (Point2D hitRelMaze) {
		MazeState wall = MazeLocalizer.getClosestWall(hitRelMaze, getWidth(), getHeight());
		try {
			hitWall(wall.x(), wall.y(), wall.dir());
			//System.err.println("Hitting " + wall.dir() + " " + wall.x() + "," + wall.y() + ", " + wall.dir());
		}
		catch (ArrayIndexOutOfBoundsException e) {
			System.err.println("out of bounds");
			System.err.println(hitRelMaze);
			System.err.println(wall);
			System.err.println();
		}
	}

	// Indicate a sonar miss on the wall at the given x, y, and direction
	public void missWall (int cellX, int cellY, Direction wallDirection) {
		//System.out.println("Missing: " + cellX + ", " + cellY + " " + wallDirection);
		getWall(cellX, cellY, wallDirection)[1]++;
	}

	// Determine whether a wall exists at the given x, y, and direction
	public boolean wallExists (int cellX, int cellY, Direction wallDirection) {
		return getWallProbability (cellX, cellY, wallDirection) >= EXIST_PROB_THRESHOLD;
	}

	// Get the existence probability of the wall at the given x, y, and direction
	// Returns a negative number if there is no data about the wall
	public double getWallProbability (int cellX, int cellY, Direction wallDirection) {
		int hits = getWallHits(cellX, cellY, wallDirection);
		int total = getWallTotal(cellX, cellY, wallDirection);
		// assume walls that have few hits or misses don't exist
		if (total < 5) {
			return -1;
		}
		return (double) hits / total;
	}

	// Get the number of sonar hits on the wall at the given x, y, and direction
	public int getWallHits (int cellX, int cellY, Direction wallDirection) {
		return getWall(cellX, cellY, wallDirection)[0];
	}

	// Get the number of sonar misses on the wall at the given x, y, and direction
	public int getWallMisses (int cellX, int cellY, Direction wallDirection) {
		return getWall(cellX, cellY, wallDirection)[1];
	}

	// Get the total number of sonar hits or misses on the wall at the given x, y, and direction
	public int getWallTotal (int cellX, int cellY, Direction wallDirection) {
		int hits = getWallHits(cellX, cellY, wallDirection);
		int misses = getWallMisses(cellX, cellY, wallDirection);
		return hits + misses;
	}

	// Gets the int[]{hits, misses} array for the wall at the given x, y, and direction
	private int[] getWall (int cellX, int cellY, Direction wallDirection) {
		switch (wallDirection) {
		case East: return verticalWalls[cellX+1][cellY];
		case West: return verticalWalls[cellX][cellY];
		case North: return horizontalWalls[cellX][cellY+1];
		case South: return horizontalWalls[cellX][cellY];
		}
		return null;
	}
	
	private boolean updateMazeWorldWall (MazeWorld mw, int x, int y, Direction d) {
		boolean changed = false;
		if (getWallProbability(x, y, d) < 0) {
			return false;
		}
		if (wallExists(x, y, d)) {
			if (!mw.isWall(x, y, d)) {
				changed = true;
			}
			mw.addWall(x, y, d);
		}
		else if (!isOutsideWall(x, y, d)) {
			if (mw.isWall(x, y, d)) {
				changed = true;
			}
			mw.removeWall(x, y, d);
		}
		return changed;
	}
	
	// Constructs a MazeWorld with walls where they are according to the grid.
	// returns whether the world changed
	public boolean updateMazeWorld (MazeWorld mw) {
		int width = mw.getWidth();
		int height = mw.getHeight();
		
		//mw.removeAllWalls();
		
		boolean changed = false;
		
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				changed |= updateMazeWorldWall(mw, x, y, South);
				changed |= updateMazeWorldWall(mw, x, y, West);
				
				if (y == height - 1) {
					changed |= updateMazeWorldWall(mw, x, y, North);
				}
				if (x == width - 1) {
					changed |= updateMazeWorldWall(mw, x, y, East);
				}
			}
		}
		
		return changed;
	}
	
	public boolean isOutsideWall(int cellX, int cellY, Direction direction) {
		if (cellY == 0 && direction == South)
			return true;
		if (cellY == mazeHeight && direction == North)
			return true;
		if (cellX == 0 && direction == West)
			return true;
		if (cellX == mazeWidth && direction == East)
			return true;
		
		return false;
	}

	// Constructs a 2 by 2 grid, puts two misses on every wall
	// For each cell, hit every surrounding wall once
	// For each cell and direction, print out x/y/direction, wall hits/misses, and wall existence
	// I'm pretty sure the output is correct, but you can manually verify it if you want.
	public static void main (String... args) {
		int size = 2;
		ProbabilisticWallGrid pwg = new ProbabilisticWallGrid(size, size);
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				pwg.hitWall(x,y,South);
				pwg.hitWall(x,y,West);
				pwg.hitWall(x,y,North);
				pwg.hitWall(x,y,East);

				pwg.missWall(x,y,South);
				pwg.missWall(x,y,West);

				if (y == size - 1) {
					pwg.missWall(x,y,North);
				}
				if (x == size - 1) {
					pwg.missWall(x,y,East);
				}
			}
		}

		for (int x = 0; x < size; x++) {
			for (int y = 0; y < size; y++) {
				for (int d = 0; d < 4; d++) {
					Direction dir = Direction.values()[d];
					int hits = pwg.getWallHits(x,y,dir);
					int total = pwg.getWallTotal(x,y,dir);
					double prob = pwg.getWallProbability(x, y, dir);
					boolean isWall = pwg.wallExists(x,y,dir);
					System.out.printf("(%d,%d,%d): %d/%d, %.2f, %b\n", x, y, d, hits, total, prob, isWall);
				}
				System.out.println();
			}
		}
	}
}