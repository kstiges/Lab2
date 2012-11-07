package edu.cmu.ri.mrpl.example;

import edu.cmu.ri.mrpl.usbCamera.*;

import javax.swing.*;
import java.awt.*;

/* This is a sample driver class for the UsbCamera Class */
public class UsbCameraDriver extends JFrame {
	private static final long serialVersionUID = 100L;

	public static void main(String args[]) {
		UsbCameraDriver go = new UsbCameraDriver();
		if(go == null);
	}

	private UsbCamera cam;

	public UsbCameraDriver() {
		super();
		
		/* Get our camera instance */
		cam = UsbCamera.getInstance();
		PicCanvas canvas = new PicCanvas();
		getContentPane().add(canvas, BorderLayout.CENTER);
		this.setSize(UsbCamera.XSIZE, UsbCamera.YSIZE);

		/* Display the window */
		this.setVisible(true);
		
		double lownr = 1;
		double highnr = 0;
		double lowng = 1;
		double highng = 0;

		/* Now Loop */
		while(true) {
			/* Take a picture */
			cam.snap();
			canvas.setImage(cam.getImage());
			canvas.setRect(UsbCamera.XSIZE/2 -5, UsbCamera.YSIZE/2 -5, 10, 10);

			for(int i=0; i<UsbCamera.XSIZE; i++) {
				for(int j=0;j<UsbCamera.YSIZE; j++) {
					try {
						cam.getRawPixel(i,j);
					} catch (Exception e) {
						System.out.println(e+" at " + i + "," + j);
					}
				}
			}

			/* Print out a random pixel */
			int pixel[] = cam.getPixel(UsbCamera.XSIZE/2, UsbCamera.YSIZE/2);
			
			double nr = ((double)pixel[UsbCamera.RED])/(pixel[UsbCamera.RED] + pixel[UsbCamera.BLUE] + pixel[UsbCamera.GREEN]);
			double nb = ((double)pixel[UsbCamera.BLUE])/(pixel[UsbCamera.RED] + pixel[UsbCamera.BLUE] + pixel[UsbCamera.GREEN]);
			double ng = ((double)pixel[UsbCamera.GREEN])/(pixel[UsbCamera.RED] + pixel[UsbCamera.BLUE] + pixel[UsbCamera.GREEN]);

			if(nr < lownr)
				lownr = nr;
			if(nr > highnr)
				highnr = nr;
			if(ng < lowng)
				lowng = ng;
			if(ng > highng)
				highng = ng;
			
			System.out.println(" r:"+pixel[UsbCamera.RED]+
					" g:"+pixel[UsbCamera.GREEN]+
					" b:"+pixel[UsbCamera.BLUE]);
			
			System.out.println("nr:" +nr+
					" ng:" +ng+
					" nb:" +nb);
			
			System.out.println(lownr+" < nr < "+highnr);
			System.out.println(lowng+" < ng < "+highng);
			//blue 0.24 < nr < 0.47
			//blue 0.20 < ng < 0.35
			//non-blue 0.176 < nr < 0.43
			//non-blue 0.21 < ng < 0.42
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {}

			canvas.unsetRect();

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {}
		}
	}
}
