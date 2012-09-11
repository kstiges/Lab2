package edu.cmu.ri.mrpl.util;

import java.awt.geom.Area;

import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

public class OccupancyGrid {
	private long[][] grid;
	private long hits;
	
	// cell dimensions in meters
	private double cellSize;
	
	public OccupancyGrid (int rows, int cols, double cellSize) {
		grid = new long[rows][cols];
		hits = 0;
		this.cellSize = cellSize;
	}
	
	public void addObstacles (Area[] cSpaceObstacles, RealPose2D robotRelWorld) {
		hits++;
	}
}
