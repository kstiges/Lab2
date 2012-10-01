package edu.cmu.ri.mrpl.util;

import static java.lang.Math.*;

import java.awt.geom.Point2D;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import static edu.cmu.ri.mrpl.maze.MazeLocalizer.*;

public class GradientDescent {
	private static final double dx = 0.01;
	private static final double dy = 0.01;
	private static final double dtheta = 0.01;
	private static final double ERROR_THRESHOLD = 0.25;

	public interface ErrorFunction {
		public double getError (double[] args);
	}

	public static void descend (ErrorFunction errFunc, double[] args) {
		double error;
		int counter = -1;
		while ((error = errFunc.getError(args)) > ERROR_THRESHOLD && counter++ > -2) {
			double[] gradient = computeGradient(errFunc, args);
			final double STEP_SIZE = 0.001;
			double dx = gradient[0] * STEP_SIZE;
			double dy = gradient[1] * STEP_SIZE;
			double dtheta = gradient[2] * STEP_SIZE;
			
			// adjust theta before x and y
			// TODO is this a good idea?
			final double DTHETA_THRESHOLD = 0.0001;
			if (abs(dtheta) < DTHETA_THRESHOLD) {
				args[0] -= dx;
				args[1] -= dy;
			}
			else {
				args[2] -= dtheta;
			}

			if (counter % 1000 == 0 && false) {
				System.err.println("counter: " + counter);
				System.err.println("error = " + error);
				System.err.println("gradient: " + gradient[0] + " " + gradient[1] + " " + gradient[2]);
				System.err.println("step: " + dx + " " + dy + " " + dtheta);
				System.err.println("args: " + args[0] + " " + args[1] + " " + args[2]);
				System.err.println();
			}
		}
	}

	public static double[] computeGradient (ErrorFunction errFunc, double[] args) {
		double oldError = errFunc.getError(args);

		args[0] += dx;
		double errXp = errFunc.getError(args);
		args[0] -= dx;

		args[1] += dy;
		double errYp = errFunc.getError(args);
		args[1] -= dy;

		args[2] += dtheta;
		double errTp = errFunc.getError(args);
		args[2] -= dtheta;

		double[] gradient = new double[3];
		gradient[0] = (errXp - oldError) / dx;
		gradient[1] = (errYp - oldError) / dy;
		gradient[2] = (errTp - oldError) / dtheta;

		return gradient;
	}

	public static void main (String... args) {
		ErrorFunction parabola = new ErrorFunction(){
			public double getError (double[] pose) {
				return abs(pose[0]-5) + abs(pose[1]-2) + abs(pose[2]-1.57);
			}
		};

		// make a 3x3 grid of points that are one wall length apart
		RingBuffer<RealPoint2D> points = new RingBuffer<RealPoint2D>(10);
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				points.add(new RealPoint2D(col*WALL_METERS, row*WALL_METERS));
			}
		}
		ErrorFunction fitter = new WallPointFitter(points);

		double[] coords = new double[]{.5, .5, 1};
		descend(fitter, coords);
		for (double d : coords) {
			System.out.println(d);
		}
		System.out.println("Error = " + fitter.getError(coords));
	}

	public static class WallPointFitter implements ErrorFunction {
		private RingBuffer<RealPoint2D> points;

		public WallPointFitter (RingBuffer<RealPoint2D> points) {
			this.points = points;
		}

		public double getError (double[] args) {
			RealPose2D pose = new RealPose2D(args[0], args[1], args[2]);
			//System.err.println("getError: pose = " + pose);
			double error = 0;

			for (int i = 0; i < points.getCapacity(); i++) {
				RealPoint2D point = points.get(i);

				// skip gaps in the point buffer
				if (point == null) {
					continue;
				}

				// transform to frame of specified pose
				Point2D transformed = pose.inverseTransform(point, null);

				// find the nearest wall
				double x = transformed.getX();
				double y = transformed.getY();
				double xWall = round(x / WALL_METERS) * WALL_METERS;
				double yWall = round(y / WALL_METERS) * WALL_METERS;
				double xDist = abs(x - xWall);
				double yDist = abs(y - yWall);
				double wallDist = min(xDist, yDist);
				// use distance squared to avoid negatives
				double wallErr = pow(wallDist, 2);

				System.err.printf("fit point #%d (%.2f,%.2f) to wall (%.2f, %.2f) %.2f m away\n", i, x, y, xWall, yWall, wallDist);

				// accumulate distances into error
				error += wallErr;
			}

			return error;
		}
	}
}
