package edu.cmu.ri.mrpl.util;

import static java.lang.Math.*;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

public class GradientDescent {
	private static final double dx = 0.01;
	private static final double dy = 0.01;
	private static final double dtheta = 0.01;
	private static final double ERROR_THRESHOLD = 0.25;
	
	public interface ErrorFunction {
		public double getError (double[] args);
	}
	
	public static void descend (ErrorFunction errFunc, double[] args) {
		while (errFunc.getError(args) > ERROR_THRESHOLD) {
			double[] gradient = computeGradient(errFunc, args);
			// XXX why multiply by 0.001?
			double dx = gradient[0] * 0.001;
			double dy = gradient[1] * 0.001;
			double dtheta = gradient[2] * 0.001;
			args[0] -= dx;
			args[1] -= dy;
			args[2] -= dtheta;
			//System.out.println(args[0] + " " + args[1] + " " + args[2] + "\n");
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
			private static final double x0 = 5;
			private static final double y0 = 2;
			private static final double th0 = 1.57;
			
			public double getError (double[] pose) {
				// TODO square instead of abs?
				return abs(pose[0]-x0) + abs(pose[1]-y0) + abs(pose[2]-th0);
			}
		};
		
		double[] coords = new double[]{50, 50, 50};
		descend(parabola, coords);
		for (double d : coords) {
			System.out.println(d);
		}
	}
}
