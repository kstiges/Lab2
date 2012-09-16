package edu.cmu.ri.mrpl;

import static java.lang.Math.*;

public class AngleMath {
	public static final double TWOPI = 2*PI;
	
	// convert angle to +/- PI
	public static double normalize (double angle) {
		angle = (angle % TWOPI + TWOPI) % TWOPI;
		if (angle > PI) {
			angle -= TWOPI;
		}
		return angle;
	}
	
	// normalized angle addition
	public static double add (double angle1, double angle2) {
		return normalize(angle1 + angle2);
	}
}
