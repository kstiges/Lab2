package edu.cmu.ri.mrpl.util;

public class AngleMath {	
	public static double roundTo2 (double angle) {
		return ((int) (angle*100)) / 100.0;
	}
}
