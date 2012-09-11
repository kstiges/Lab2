package edu.cmu.ri.mrpl.util;

import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

public class OccupancyGrid {
	private long[][] grid;
	private int rows;
	private int cols;
	private RealPose2D worldRelGrid;
	private long hits;
	
	private static final double OCCUPANCY_THRESHOLD = 0.5;
	
	// cell dimensions in meters
	private double cellWidth;
	private double cellHeight;
	
	public OccupancyGrid (int rows, int cols, double cellSize) {
		this.rows = rows;
		this.cols = cols;
		worldRelGrid = new RealPose2D(cols/2, rows/2, 0);
		grid = new long[rows][cols];
		hits = 0;
		// use square cells for now
		this.cellWidth = cellSize;
		this.cellHeight = cellSize;
	}
	
	private int getRow (double yCoord) {
		return (int) (yCoord / cellHeight);
	}
	
	private int getCol (double xCoord) {
		return (int) (xCoord / cellWidth);
	}
	
	private boolean hasRow (int rowIndex) {
		return 0 <= rowIndex && rowIndex < rows;
	}
	
	private boolean hasCol (int colIndex) {
		return 0 <= colIndex && colIndex < cols;
	}
	
	private boolean addGridObstacle (Area gridObstacle) {
		// determine which cells lie within the obstacle bounding box
		// and need to be checked for intersection with the object
		Rectangle2D bounds = gridObstacle.getBounds2D();
		int startRow = getRow(bounds.getMinY());
		int endRow = getRow(bounds.getMaxY());
		int startCol = getCol(bounds.getMinX());
		int endCol = getCol(bounds.getMaxX());
		
		// if the obstacle lies outside of the grid, we can't add it
		// indicate this by returning false
		if (!hasRow(startRow) || !hasRow(endRow)
				|| !hasCol(startCol) || !hasCol(endCol)) {
			return false;
		}
		
		// otherwise, iterate through the bounding box cells and check for intersection
		for (int row = startRow; row <= endRow; row++) {
			for (int col = startCol; col <= endCol; col++) {
				Area cellArea = new Area(new Rectangle2D.Double(
						col*cellWidth, row*cellHeight, cellWidth, cellHeight));
				cellArea.intersect(gridObstacle);
				if (!cellArea.isEmpty()) {
					grid[row][col]++;
				}
			}
		}
		
		return true;
	}
	
	public void addRobotObstacles (Area[] robotObstacles, RealPose2D robotRelWorld) {
		hits++;
		for (Area robotObstacle : robotObstacles) {
			// XXX might not need to clone if Area array is not used after this
			Area worldObstacle = (Area) robotObstacle.clone();
			worldObstacle.transform(robotRelWorld);
			Area gridObstacle = worldObstacle;
			gridObstacle.transform(worldRelGrid);
			addGridObstacle(gridObstacle);
		}
	}
	
	public boolean gridCellOccupied (int row, int col) {
		return (grid[row][col] / hits) > OCCUPANCY_THRESHOLD;
	}
}
