package edu.cmu.ri.mrpl.maze;

public class ProbabilisticWallGrid {
	// maze dimensions
	private int width;
	private int height;

	// Hold hits/misses for horizontal and vertical walls separately.
	// Indexed by x, then y, then hits (0) or misses (1)
	private int[][][] horizontalWalls;
	private int[][][] verticalWalls;

	// TODO ensure these match MazeState.Direction enum values
	private static final int EAST = 0;
	private static final int NORTH = 1;
	private static final int WEST = 2;
	private static final int SOUTH = 3;

	// To be said to exist, a wall must have at least this probability of existence. Adjust this as needed.
	private static final double EXIST_PROB_THRESHOLD = 0.6;

	// Constructs a wall grid for a maze of given width/height
	public ProbabilisticWallGrid (int width, int height) {
		this.width = width;
		this.height = height;
		
		horizontalWalls = new int[width][height+1][2];
		verticalWalls = new int[width+1][height][2];
	}

	// Indicate a sonar hit on the wall at the given x, y, and direction
	public void hitWall (int cellX, int cellY, int wallDirection) {
		getWall(cellX, cellY, wallDirection)[0]++;
	}

	// Indicate a sonar miss on the wall at the given x, y, and direction
	public void missWall (int cellX, int cellY, int wallDirection) {
		getWall(cellX, cellY, wallDirection)[1]++;
	}

	// Determine whether a wall exists at the given x, y, and direction
	public boolean isWall (int cellX, int cellY, int wallDirection) {
		return getWallProbability (cellX, cellY, wallDirection) >= EXIST_PROB_THRESHOLD;
	}

	// Get the existence probability of the wall at the given x, y, and direction
	public double getWallProbability (int cellX, int cellY, int wallDirection) {
		int hits = getWallHits(cellX, cellY, wallDirection);
		int total = getWallTotal(cellX, cellY, wallDirection);
		return (double) hits / total;
	}

	// Get the number of sonar hits on the wall at the given x, y, and direction
	public int getWallHits (int cellX, int cellY, int wallDirection) {
		return getWall(cellX, cellY, wallDirection)[0];
	}

	// Get the number of sonar misses on the wall at the given x, y, and direction
	public int getWallMisses (int cellX, int cellY, int wallDirection) {
		return getWall(cellX, cellY, wallDirection)[1];
	}

	// Get the total number of sonar hits or misses on the wall at the given x, y, and direction
	public int getWallTotal (int cellX, int cellY, int wallDirection) {
		int hits = getWallHits(cellX, cellY, wallDirection);
		int misses = getWallMisses(cellX, cellY, wallDirection);
		return hits + misses;
	}

	// Gets the int[]{hits, misses} array for the wall at the given x, y, and direction
	private int[] getWall (int cellX, int cellY, int wallDirection) {
		if (wallDirection == EAST) {
			return verticalWalls[cellX+1][cellY];
		}
		else if (wallDirection == WEST) {
			return verticalWalls[cellX][cellY];
		}
		else if (wallDirection == NORTH) {
			return horizontalWalls[cellX][cellY+1];
		}
		else if (wallDirection == SOUTH) {
			return horizontalWalls[cellX][cellY];
		}
		else {
			return null;
		}
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
				pwg.hitWall(x,y,SOUTH);
				pwg.hitWall(x,y,WEST);
				pwg.hitWall(x,y,NORTH);
				pwg.hitWall(x,y,EAST);

				pwg.missWall(x,y,SOUTH);
				pwg.missWall(x,y,WEST);

				if (y == size - 1) {
					pwg.missWall(x,y,NORTH);
				}
				if (x == size - 1) {
					pwg.missWall(x,y,EAST);
				}
			}
		}

		for (int x = 0; x < size; x++) {
			for (int y = 0; y < size; y++) {
				for (int d = 0; d < 4; d++) {
					int hits = pwg.getWallHits(x,y,d);
					int total = pwg.getWallTotal(x,y,d);
					boolean isWall = pwg.isWall(x,y,d);
					System.out.printf("(%d,%d,%d): %d/%d, %b\n", x, y, d, hits, total, isWall);
				}
				System.out.println();
			}
		}
	}
}